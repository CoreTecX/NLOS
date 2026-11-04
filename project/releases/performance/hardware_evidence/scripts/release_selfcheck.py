"""Read-only validation of this release. Never access board hardware."""
from pathlib import Path
import json,hashlib,struct,subprocess,sys,re
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    manifest=json.loads((ROOT/'release_manifest.json').read_text('utf-8'))
    assert manifest['release_id']=='20261015-rp2-m8-v1' and manifest['layout_version']==0x32525001
    assert manifest['offline_acceptance'] is True and manifest['physical_board_test'] is False
    for rel,entry in manifest['files'].items():
        p=ROOT/rel
        if not p.is_file() or p.stat().st_size!=entry['bytes'] or sha(p)!=entry['sha256']:
            raise RuntimeError('Release file mismatch: '+rel)
    audit=json.loads((ROOT/'reports/acceptance.json').read_text('utf-8'));assert audit['offline_gates_passed']
    rows=json.loads((ROOT/'data/manifest.json').read_text('utf-8'));assert len(rows)==15 and len({r['geometry'] for r in rows})==7
    for r in rows:
        for key in ('model','measurement','golden'):
            assert sha(ROOT/'data'/r[key]['file'])==r[key]['sha256']
    for rel,expected in json.loads((ROOT/'reports/rtl_lite_sources.json').read_text()).items():
        assert sha(ROOT/'build/rtl_lite'/rel)==expected,rel
    elf=(ROOT/'bin/ps_full_aarch64').read_bytes();assert elf[:6]==b'\x7fELF\x02\x01' and struct.unpack_from('<H',elf,18)[0]==183
    off=struct.unpack_from('<Q',elf,32)[0];size,count=struct.unpack_from('<HH',elf,54)
    assert not {2,3}.intersection(struct.unpack_from('<I',elf,off+i*size)[0] for i in range(count))
    regs=(ROOT/'full_gemv_hls/solution1/impl/ip/drivers/full_gemv_v2_0/src/xfull_gemv_hw.h').read_text()
    for name,value in [('MATRIX_DATA',0x10),('INPUT_R_DATA',0x1c),('OUTPUT_R_DATA',0x28),('N_DATA',0x34),('AP_RETURN',0x3c),('LAYOUT_DATA',0x44)]:
        assert int(re.search('XFULL_GEMV_CONTROL_ADDR_'+name+r'\s+(0x[0-9a-fA-F]+)',regs)[1],16)==value
    for backend in ('cpu','pl'):
        subprocess.run([sys.executable,str(ROOT/'scripts/run_target.py'),'--backend',backend,'--all','--preflight-only'],check=True)
    print('M4_M56_RELEASE_SELFCHECK_PASS files='+str(len(manifest['files']))+' cases=15 hardware_untouched=1')
if __name__=='__main__':main()
