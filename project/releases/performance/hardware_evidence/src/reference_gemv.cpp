#include <cstdint>
// One matrix load serves all three colour channels. Finite FP64 inputs only.
// The host and RTL use the same 16-way summation order, without reassociation.
extern "C" unsigned reference_gemv(const double *matrix, const double *input,
                              double *output, unsigned n) {
#pragma HLS INTERFACE m_axi port=matrix offset=slave bundle=mat depth=1089936 max_read_burst_length=64 num_read_outstanding=8
#pragma HLS INTERFACE m_axi port=input offset=slave bundle=vec depth=3132
#pragma HLS INTERFACE m_axi port=output offset=slave bundle=vec depth=3132
#pragma HLS INTERFACE s_axilite port=matrix bundle=control offset=0x10
#pragma HLS INTERFACE s_axilite port=input bundle=control offset=0x1c
#pragma HLS INTERFACE s_axilite port=output bundle=control offset=0x28
#pragma HLS INTERFACE s_axilite port=n bundle=control offset=0x34
#pragma HLS INTERFACE s_axilite port=return bundle=control
    if (n == 0 || n > 1044) return 0x46500101u;
    double x[1044][3];
#pragma HLS ARRAY_PARTITION variable=x complete dim=2
    for (unsigned j=0;j<n;++j)
        for (unsigned c=0;c<3;++c) {
#pragma HLS PIPELINE II=1
            x[j][c]=input[3*j+c];
        }
    for (unsigned i=0;i<n;++i) {
        double partial[16][3];
#pragma HLS ARRAY_PARTITION variable=partial complete dim=0
        for (unsigned s=0;s<16;++s) {
#pragma HLS UNROLL
            for (unsigned c=0;c<3;++c) {
#pragma HLS UNROLL
                partial[s][c]=0;
            }
        }
        for (unsigned j=0;j<n;++j) {
#pragma HLS PIPELINE II=1
#pragma HLS DEPENDENCE variable=partial type=inter direction=RAW distance=16 true
            const double a=matrix[i*n+j];
            for (unsigned c=0;c<3;++c) {
#pragma HLS UNROLL
                const double p=a*x[j][c];
                partial[j&15][c]=partial[j&15][c]+p;
            }
        }
        for (unsigned c=0;c<3;++c) {
            double sum=0;
            for (unsigned s=0;s<16;++s) sum+=partial[s][c];
            output[i*3+c]=sum;
        }
    }
    return 0x46500100u;
}
