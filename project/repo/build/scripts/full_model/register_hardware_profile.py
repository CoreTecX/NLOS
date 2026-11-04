"""Register a fully verified P5/P6 build for a matched candidate package.

This writes metadata only. It does not publish a final P11 release or use a board.
"""
from pathlib import Path
import argparse,json,hashlib,sys,re
sys.dont_write_bytecode=True
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--build',type=Path,required=True)
    ap.add_argument('--software-verification',type=Path,required=True)
    ap.add_argument('--catalog',type=Path,required=True)
    args=ap.parse_args();build=args.build.resolve()
    def read(rel):return json.loads((build/rel).read_text('utf-8'))
    contract=read('reports/contract_audit.json');clock=read('reports/clock_contract.json')
    for rel in ['reports/contract_audit.json','reports/clock_contract.json','reports/rtl_parallel.json','reports/reset_integration.json','reports/rtl_source_pairing.json','reports/rtl_float_model_pairing.json','reports/rtl_shard_source_audit.json','reports/physical_source_pairing.json','reports/pressure_v4/result.json']:
        if read(rel).get('passed') is not True:raise RuntimeError('Incomplete evidence: '+rel)
    timing=read('reports/routed_audit.json')
    if read('reports/pressure_v4/result.json').get('fifo_full_abort_reset') is not True:
        raise RuntimeError('Buffered FIFO reset recovery has not passed')
    if timing['wns_ns']<0 or timing['whs_ns']<0 or timing['drc_errors'] or timing['black_boxes'] or not timing['expected_clock_found']:
        raise RuntimeError('Routed implementation did not pass')
    unchecked=re.findall(r'checking (\w+) \((\d+)\)',(build/'reports/check_timing.rpt').read_text())
    if not unchecked or any(int(n) for _,n in unchecked):raise RuntimeError('Unresolved timing checks')
    software=json.loads(args.software_verification.read_text('utf-8'))
    if software.get('passed') is not True:raise RuntimeError('Software verification incomplete')
    source=Path(software['staging']);bins={}
    for item in software['build']['jobs']:
        rel=item['output'];p=source/rel
        if 'aarch64' not in p.name or not p.name.startswith('ps_full_'):continue
        if sha(p)!=item['sha256']:raise RuntimeError('Software artifact changed: '+str(p))
        bins[p.name]=item['sha256']
    if not bins:raise RuntimeError('No verified ARM applications')
    # The same kernel C++ and immutable ABI headers must underpin the HLS and host test.
    for name in ['full_gemv.cpp','gemv_contract.hpp','row_pair.hpp','reference_gemv.cpp']:
        if sha(source/'src'/name)!=sha(build/'src'/name):raise RuntimeError('Mixed algorithm sources: '+name)
    value=dict(hardware_release='20261016-rp2-p56-v1',layout_abi='0x32525001',profiles={})
    if args.catalog.exists():
        value=json.loads(args.catalog.read_text('utf-8'))
        if value.get('hardware_release')!='20261016-rp2-p56-v1':raise RuntimeError('Wrong catalog')
    entry=dict(offline_passed=True,physical_board_test=False,actual_mhz=clock['actual_mhz'],pl0_value=int(clock['pl0_value'],16),
        bitstream_sha256=sha(build/'bin/full_board_fan_safe.bit'),xsa_sha256=sha(build/'bin/full_board.xsa'),routed_checkpoint_sha256=sha(build/'bin/full_board_routed.dcp'),
        elf_sha256=bins,columns_ii=contract['columns_ii'],axi_burst=contract['axi_burst'],axi_outstanding=contract['axi_outstanding'],add_latency_cycles=contract['add_latencies_cycles'],wns_ns=timing['wns_ns'],whs_ns=timing['whs_ns'],
        evidence_sha256={p.name:sha(p) for p in (build/'reports').glob('*.json') if p.name in ['rtl_parallel.json','routed_audit.json','clock_contract.json','reset_integration.json','contract_audit.json']})
    value['profiles'][clock['profile']]=entry;value.pop('status',None)
    args.catalog.parent.mkdir(parents=True,exist_ok=True);args.catalog.write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')
    print('VERIFIED_PROFILE_REGISTERED',clock['profile'])
if __name__=='__main__':main()
