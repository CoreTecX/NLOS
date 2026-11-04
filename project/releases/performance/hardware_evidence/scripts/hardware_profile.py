"""Clock contract for the P5/P6 candidate. Pure validation is safe on the host."""
TOKEN='20261016-rp2-p56-v1'
PL0=0xff5e00c0
PL0_MASK=0x013f3f07
# The ALINX preset produces IOPLL=33.333333 MHz * 90 / 2 = 1500 MHz.
# Use that same source in BD and at runtime; never reprogram a shared PS PLL.
PROFILES={
    'p56_100':dict(mhz=100.0,source='IOPLL',divisor0=15,divisor1=1),
    'p56_125':dict(mhz=125.0,source='IOPLL',divisor0=12,divisor1=1),
    'p56_150':dict(mhz=150.0,source='IOPLL',divisor0=10,divisor1=1),
    'p56_150_b128':dict(mhz=150.0,source='IOPLL',divisor0=10,divisor1=1),
}
def profile_for(manifest):
    if manifest.get('release_id')!=TOKEN or manifest.get('hardware_release')!=TOKEN:
        raise RuntimeError('Wrong P5/P6 hardware identity')
    name=manifest.get('hardware_profile')
    if name not in PROFILES:raise RuntimeError('Missing/unknown hardware clock profile')
    return dict(PROFILES[name],name=name)
def expected_pl0(profile):
    if profile.get('name') not in PROFILES or profile!=dict(PROFILES[profile['name']],name=profile['name']):
        raise RuntimeError('Modified clock profile')
    return 0x01000000 | (profile['divisor1']<<16) | (profile['divisor0']<<8)
def verify_binding(manifest,catalog,binary):
    profile=profile_for(manifest)
    if catalog.get('hardware_release')!=TOKEN or catalog.get('layout_abi')!='0x32525001':
        raise RuntimeError('Wrong approved hardware catalog')
    entry=catalog.get('profiles',{}).get(profile['name'])
    if not entry or entry.get('offline_passed') is not True:
        raise RuntimeError('Hardware profile has no completed offline acceptance')
    if entry.get('actual_mhz')!=profile['mhz'] or entry.get('pl0_value')!=expected_pl0(profile):
        raise RuntimeError('Approved hardware clock disagrees with launcher')
    files=manifest.get('files',{})
    if files.get('bin/full_board_fan_safe.bit',{}).get('sha256')!=entry.get('bitstream_sha256'):
        raise RuntimeError('Bitstream does not belong to selected clock profile')
    expected=entry.get('elf_sha256',{}).get(binary)
    if not expected or files.get('bin/'+binary,{}).get('sha256')!=expected:
        raise RuntimeError('ELF does not belong to approved hardware package')
    return profile
def check_iopll(read):
    ctrl=read(0xff5e0020);frac=read(0xff5e0028);status=read(0xff5e0040)
    if ctrl&0x00717f09!=0x00015a00 or frac&0x80000000 or not status&1:
        raise RuntimeError('IOPLL does not match the routed 1500 MHz preset; use a matching boot image before loading PL')
    return dict(ctrl=ctrl,frac=frac,status=status,nominal_mhz=1500.0)
def configure_clock(read,mask,pause,profile):
    value=expected_pl0(profile)
    observed=check_iopll(read) # all unsupported source states fail before writes
    # Caller has already held the PL in reset and excluded other applications.
    mask(PL0,0x01000000,0);pause(.001)
    mask(PL0,PL0_MASK,value&~0x01000000);pause(.001)
    mask(PL0,0x01000000,0x01000000);pause(.001)
    if read(PL0)&PL0_MASK!=value:raise RuntimeError('PL0 clock readback mismatch; keep PL reset')
    observed.update(profile=profile,pl0=value)
    return observed
