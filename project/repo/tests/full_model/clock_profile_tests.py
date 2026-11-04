"""Exercise real clock decisions with fake register I/O; never touches hardware."""
from pathlib import Path
import sys,json
sys.dont_write_bytecode=True
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'scripts'))
import hardware_profile as hw
def main():
    checks=[]
    for name in hw.PROFILES:
        p=hw.profile_for(dict(release_id=hw.TOKEN,hardware_release=hw.TOKEN,hardware_profile=name))
        registers={0xff5e0020:0x15a00,0xff5e0028:0,0xff5e0040:1,hw.PL0:0x81010a02}
        writes=[]
        def mask(addr,bits,value):
            registers[addr]=(registers[addr]&~bits)|(value&bits);writes.append((addr,bits,value))
        result=hw.configure_clock(registers.__getitem__,mask,lambda _:None,p)
        assert 1500/(p['divisor0']*p['divisor1'])==p['mhz']
        assert registers[hw.PL0]&hw.PL0_MASK==hw.expected_pl0(p)
        assert registers[hw.PL0]&0x80000000
        assert len(writes)==3 and not writes[0][2]
        checks.append(name+'_valid_and_reserved_bits_preserved')
        for label,addr,value in [('wrong_fbdiv',0xff5e0020,0x14800),('bypass',0xff5e0020,0x15a08),('reset',0xff5e0020,0x15a01),('wrong_source',0xff5e0020,0x415a00),('fractional',0xff5e0028,0x80000000),('unlocked',0xff5e0040,0)]:
            saved=registers[addr];registers[addr]=value;writes.clear()
            try:hw.configure_clock(registers.__getitem__,mask,lambda _:None,p)
            except RuntimeError:pass
            else:raise AssertionError(label)
            assert not writes;registers[addr]=saved;checks.append(name+'_'+label)
        try:hw.configure_clock(registers.__getitem__,lambda *args:None,lambda _:None,dict(p,divisor0=1))
        except RuntimeError:checks.append(name+'_modified_profile_rejected')
        else:raise AssertionError('modified profile accepted')
        registers[hw.PL0]=0
        try:hw.configure_clock(registers.__getitem__,lambda *args:None,lambda _:None,p)
        except RuntimeError:checks.append(name+'_failed_readback_rejected')
        else:raise AssertionError('write failure accepted')
    manifest=dict(release_id=hw.TOKEN,hardware_release=hw.TOKEN,hardware_profile='p56_100',files={
        'bin/full_board_fan_safe.bit':dict(sha256='bit100'),'bin/app':dict(sha256='elf1')})
    catalog=dict(hardware_release=hw.TOKEN,layout_abi='0x32525001',profiles={
        'p56_100':dict(offline_passed=True,actual_mhz=100.,pl0_value=0x01010f00,bitstream_sha256='bit100',elf_sha256={'app':'elf1'}),
        'p56_150':dict(offline_passed=True,actual_mhz=150.,pl0_value=0x01010a00,bitstream_sha256='bit150',elf_sha256={'app':'elf1'})})
    hw.verify_binding(manifest,catalog,'app');checks.append('matching_approved_hardware_binding')
    from copy import deepcopy
    for name in ['clock_name_only','unapproved','wrong_bitstream','wrong_elf','wrong_routed_frequency','wrong_divider','unfinished_acceptance']:
        m=deepcopy(manifest);c=deepcopy(catalog)
        if name=='clock_name_only':m['hardware_profile']='p56_150'
        elif name=='unapproved':m['hardware_profile']='p56_125'
        elif name=='wrong_bitstream':m['files']['bin/full_board_fan_safe.bit']['sha256']='different'
        elif name=='wrong_elf':m['files']['bin/app']['sha256']='different'
        elif name=='wrong_routed_frequency':c['profiles']['p56_100']['actual_mhz']=125.
        elif name=='wrong_divider':c['profiles']['p56_100']['pl0_value']=0x01010a00
        elif name=='unfinished_acceptance':c['profiles']['p56_100']['offline_passed']=False
        try:hw.verify_binding(m,c,'app')
        except RuntimeError:checks.append(name+'_rejected')
        else:raise AssertionError(name+' accepted')
    result=dict(passed=True,checks=checks,physical_board_test=False)
    (root/'reports/clock_profile_tests.json').write_text(json.dumps(result,indent=2))
    print('CLOCK_PROFILE_TESTS_PASS',len(checks))
if __name__=='__main__':main()
