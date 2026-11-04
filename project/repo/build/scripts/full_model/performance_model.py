"""Conditional sensitivity model. No ARM/DDR measurement is invented."""
import argparse,csv,json,itertools
from pathlib import Path
MATRIX_BYTES=1044*1044*8
PAIR_BYTES=2*MATRIX_BYTES
IDEAL_COMPUTE_CYCLES=522*1044
def evaluate(frequency_mhz,bandwidth_gbs,cache_fraction=0,observed_extra_cycles=91525):
    f=frequency_mhz*1e6
    b=min(bandwidth_gbs*1e9,16*f) # existing 128-bit HP0 path
    traffic=MATRIX_BYTES*(1-cache_fraction)
    transfer=traffic/b
    compute=IDEAL_COMPUTE_CYCLES/f
    return dict(frequency_mhz=frequency_mhz,assumed_effective_bandwidth_gbs=b/1e9,
        cached_fraction_of_both_matrices=cache_fraction,cache_bytes=PAIR_BYTES*cache_fraction,
        single_gemv_lower_bound_seconds=max(compute,transfer),
        single_gemv_calibrated_scenario_seconds=max(compute,transfer)+observed_extra_cycles/f,
        single_gemv_serialized_scenario_seconds=compute+transfer+observed_extra_cycles/f,
        two_gemv_lower_bound_seconds=2*max(compute,transfer))
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    rows=[evaluate(f,b,c) for f,b,c in itertools.product([100,125,150],[.4,.8,1.2,1.6],[0,.1,.2])]
    with (a.output/'sensitivity.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
    iteration_rows=[];overlap_rows=[]
    for row in rows:
        for iterations in [1000,2900,8200,20000]:
            iteration_rows.append(dict(frequency_mhz=row['frequency_mhz'],assumed_effective_bandwidth_gbs=row['assumed_effective_bandwidth_gbs'],cache_fraction=row['cached_fraction_of_both_matrices'],hypothetical_iterations=iterations,
                matrix_only_loop_lower_bound_seconds=iterations*row['two_gemv_lower_bound_seconds'],
                matrix_only_calibrated_scenario_seconds=iterations*2*row['single_gemv_calibrated_scenario_seconds']))
        for dct_ms in [.1,.5,2.,8.]:
            # Sensitivity points, not inferred ARM bounds. Omitted work is explicitly not included.
            gemv=row['single_gemv_calibrated_scenario_seconds'];dct=dct_ms/1000.
            overlap_rows.append(dict(frequency_mhz=row['frequency_mhz'],assumed_effective_bandwidth_gbs=row['assumed_effective_bandwidth_gbs'],cache_fraction=row['cached_fraction_of_both_matrices'],hypothetical_arm_dct_ms=dct_ms,
                synchronous_partial_iteration_ms=(dct+2*gemv)*1000,
                overlap_partial_iteration_ms=(max(dct,gemv)+gemv)*1000,
                maximum_overlap_saving_ms=min(dct,gemv)*1000))
    for name,table in [('iteration_sensitivity.csv',iteration_rows),('critical_path_sensitivity.csv',overlap_rows)]:
        with (a.output/name).open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=table[0]);w.writeheader();w.writerows(table)
    model=dict(matrix_bytes=MATRIX_BYTES,two_matrices_bytes=PAIR_BYTES,vector_bytes=1044*3*8,
        nominal_100mhz_128bit_gbs=1.6,ideal_matrix_cycles=IDEAL_COMPUTE_CYCLES,
        historical_rtl_cycles=636493,historical_rtl_case_includes_control_polling=True,
        fixed_condition_speedup_ceiling=636493/IDEAL_COMPUTE_CYCLES,
        board_measurements=dict(arm_dct_seconds=None,arm_vector_seconds=None,arm_preprocess_seconds=None,ddr_bandwidth=None,copy_seconds=None,submit_wait_seconds=None,linux_image=None,cpu_frequency=None),
        formulas=dict(synchronous_iteration='rhs + DCT + KT(copy/submit/wait/copy) + spectral_scale + K(copy/submit/wait/copy) + vector_update + amortized_residual',
            future_overlap_ideal='rhs + max(DCT, KT_inflight) + spectral_scale + K + vector_update + amortized_residual; add submit/copy work not included in KT_inflight; contention may increase both',
            cold_new_process='launcher/preflight/selftest + optional hardware setup + load/CRC/diagnostics/h + packing + sum(iteration_cost) + output + process overhead',
            steady_state='same-process reused model/backend only if such API is implemented; current CLI loads every invocation; inner solver iteration is the available steady loop',
            iteration_count='sum by observed rho/iteration path; not a promised multiplicative speedup',
            cache='warm transfer = (1-c)*bytes; first fill still pays full traffic; compute lower bound unchanged'),
        bounds='Lower bound conditional on stated clock/interface and cache assumptions. Calibrated/serialized columns are sensitivity scenarios, NOT proven latency upper bounds. Real OS/DDR wait has no finite bound from offline evidence.',
        sensitivity_scope='36 matrix scenarios; iteration and DCT critical-path tables each144 rows. DCT0.1/0.5/2/8ms are hypothetical points, not measured or guaranteed ARM bounds. RHS/scale/vector/residual/copy/submit and all startup work are omitted from these partial-loop tables and remain unknown for the target.',
        bram=dict(total_36k_blocks=912,baseline_used=15,raw_remaining_bytes=(912-15)*4608,note='raw capacity only; ports, ECC, fragmentation, timing and control consume budget; full K+KT cannot fit'),
        no_board_access=True,no_candidate_optimization_executed=True)
    (a.output/'model.json').write_text(json.dumps(model,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
