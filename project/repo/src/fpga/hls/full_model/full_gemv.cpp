#include "gemv_contract.hpp"
#if defined(__SYNTHESIS__) || defined(PS_HLS_CSIM)
#include <hls_stream.h>
template<class T> using PartialStream = hls::stream<T>;
#else
#include <deque>
// Sequential host model only. RTL verifies the finite depth and backpressure.
template<class T> class PartialStream {
    std::deque<T> queue;
public:
    void write(const T& value){queue.push_back(value);}
    T read(){T value=queue.front();queue.pop_front();return value;}
};
#endif
#ifndef PS_AXI_BURST
#define PS_AXI_BURST 64
#endif
#ifndef PS_AXI_OUTSTANDING
#define PS_AXI_OUTSTANDING 8
#endif
struct PartialPair {double values[2][16][3];};
struct PartialRow {double values[16][3];};

// Producer exclusively owns its 96 partial sums until the blocking write.
// Every row retains the original 16-slot recurrence and FP64 operation order.
static void compute_pairs(const ps_layout::Pair* matrix,const double x[1044][3],
                          unsigned n,PartialStream<PartialRow> completed[2]){
#pragma HLS INLINE off
#pragma HLS ARRAY_PARTITION variable=x complete dim=2
    ROW_PAIRS: for(unsigned r=0;r<n;r+=2){
        PartialPair partial;
#pragma HLS ARRAY_PARTITION variable=partial.values complete dim=0
        for(unsigned lane=0;lane<2;++lane){
#pragma HLS UNROLL
            for(unsigned s=0;s<16;++s){
#pragma HLS UNROLL
                for(unsigned c=0;c<3;++c){
#pragma HLS UNROLL
                    partial.values[lane][s][c]=0.;
                }
            }
        }
        COLUMNS: for(unsigned j=0;j<n;++j){
#pragma HLS PIPELINE II=1
#pragma HLS DEPENDENCE variable=partial.values type=inter direction=RAW distance=16 true
            const ps_layout::Pair p=matrix[(r/2)*n+j];
            for(unsigned lane=0;lane<2;++lane){
#pragma HLS UNROLL
                const double a=lane==0?p.lo:p.hi;
                for(unsigned c=0;c<3;++c){
#pragma HLS UNROLL
                    const double product=a*x[j][c];
                    partial.values[lane][j&15][c]=partial.values[lane][j&15][c]+product;
                }
            }
        }
        for(unsigned lane=0;lane<2;++lane){
#pragma HLS UNROLL
            PartialRow row;
#pragma HLS ARRAY_PARTITION variable=row.values complete dim=0
            for(unsigned slot=0;slot<16;++slot){
#pragma HLS UNROLL
                for(unsigned c=0;c<3;++c){
#pragma HLS UNROLL
                    row.values[slot][c]=partial.values[lane][slot][c];
                }
            }
            completed[lane].write(row);
        }
    }
}

// Consumer owns each token after read; the producer can compute the next pair.
static void reduce_pairs(PartialStream<PartialRow> completed[2],double* output,unsigned n){
#pragma HLS INLINE off
    REDUCE_PAIRS: for(unsigned r=0;r<n;r+=2){
        PartialPair partial;
        for(unsigned lane=0;lane<2;++lane){
#pragma HLS UNROLL
            const PartialRow row=completed[lane].read();
#pragma HLS ARRAY_PARTITION variable=row.values complete dim=0
            for(unsigned slot=0;slot<16;++slot){
#pragma HLS UNROLL
                for(unsigned c=0;c<3;++c){
#pragma HLS UNROLL
                    partial.values[lane][slot][c]=row.values[slot][c];
                }
            }
        }
#pragma HLS ARRAY_PARTITION variable=partial.values complete dim=0
        for(unsigned c=0;c<3;++c){
            double sum[2]={0.,0.};
#pragma HLS ARRAY_PARTITION variable=sum complete
            for(unsigned s=0;s<16;++s){
                for(unsigned lane=0;lane<2;++lane){
#pragma HLS UNROLL
                    sum[lane]+=partial.values[lane][s][c];
                }
            }
            output[r*3+c]=sum[0];
            if(r+1<n)output[(r+1)*3+c]=sum[1];
        }
    }
}
static void run_pairs(const ps_layout::Pair* matrix,const double x[1044][3],
                      double* output,unsigned n){
#pragma HLS INLINE off
#pragma HLS DATAFLOW
#pragma HLS ARRAY_PARTITION variable=x complete dim=2
    PartialStream<PartialRow> completed[2];
#pragma HLS ARRAY_PARTITION variable=completed complete
#pragma HLS STREAM variable=completed depth=2
    compute_pairs(matrix,x,n,completed);
    reduce_pairs(completed,output,n);
}

extern "C" unsigned full_gemv(const ps_layout::Pair* matrix,const double* input,
                             double* output,unsigned n,unsigned layout){
#pragma HLS INTERFACE m_axi port=matrix offset=slave bundle=mat depth=544968 max_read_burst_length=PS_AXI_BURST num_read_outstanding=PS_AXI_OUTSTANDING
#pragma HLS AGGREGATE variable=matrix compact=none
#pragma HLS INTERFACE m_axi port=input offset=slave bundle=vec depth=3132 max_widen_bitwidth=64
#pragma HLS INTERFACE m_axi port=output offset=slave bundle=vec depth=3132 max_widen_bitwidth=64
#pragma HLS INTERFACE s_axilite port=matrix bundle=control offset=0x10
#pragma HLS INTERFACE s_axilite port=input bundle=control offset=0x1c
#pragma HLS INTERFACE s_axilite port=output bundle=control offset=0x28
#pragma HLS INTERFACE s_axilite port=n bundle=control offset=0x34
#pragma HLS INTERFACE s_axilite port=layout bundle=control offset=0x44
#pragma HLS INTERFACE s_axilite port=return bundle=control offset=0x3c
    if(layout!=ps_layout::VERSION)return ps_layout::BAD_LAYOUT;
    if(n==0 || n>ps_layout::MAX_N)return ps_layout::BAD_N;
    double x[1044][3];
#pragma HLS ARRAY_PARTITION variable=x complete dim=2
    for(unsigned j=0;j<n;++j)for(unsigned c=0;c<3;++c){
#pragma HLS PIPELINE II=1
        x[j][c]=input[3*j+c];
    }
    run_pairs(matrix,x,output,n);
    return ps_layout::OK;
}
