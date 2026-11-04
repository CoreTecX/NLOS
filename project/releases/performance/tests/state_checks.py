"""Adversarial checks for independent PFS auditing, without board access."""
from pathlib import Path
import argparse,importlib.util,json,struct,zlib,tempfile,shutil

def main():
    p=argparse.ArgumentParser();p.add_argument('--model',type=Path,required=True);p.add_argument('--measurement',type=Path,required=True);p.add_argument('--prefix',type=Path,required=True);p.add_argument('--audit',type=Path,required=True);p.add_argument('--report',type=Path,required=True);a=p.parse_args()
    spec=importlib.util.spec_from_file_location('audit',a.audit);audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
    assert audit.evaluate(a.model,a.measurement,a.prefix)['passed'];checks=['valid_state']
    with tempfile.TemporaryDirectory(prefix='ps_pfs_',dir='D:/Temp') as name:
        prefix=Path(name)/'sample';shutil.copy2(a.prefix.with_suffix('.pfo'),prefix.with_suffix('.pfo'))
        original=a.prefix.with_suffix('.pfs').read_bytes()
        def reject(label,change,crc=True):
            raw=bytearray(original);change(raw)
            if crc:struct.pack_into('<I',raw,32,zlib.crc32(raw[64:]))
            prefix.with_suffix('.pfs').write_bytes(raw)
            try:passed=audit.evaluate(a.model,a.measurement,prefix)['passed']
            except (ValueError,IndexError,KeyError):passed=False
            assert not passed,label;checks.append(label)
        reject('corrupt_crc',lambda b:b.__setitem__(100,b[100]^1),False)
        reject('wrong_identity',lambda b:b.__setitem__(40,b[40]^1))
        reject('wrong_length',lambda b:b.extend(b'12345678'))
        for value,label in [(float('nan'),'nan'),(float('inf'),'inf')]:reject(label,lambda b:struct.pack_into('<d',b,64+7*8,value))
        reject('fake_logged_primal',lambda b:struct.pack_into('<d',b,64+2*8,0.))
        reject('fake_logged_dual',lambda b:struct.pack_into('<d',b,64+3*8,1.))
        reject('relaxed_threshold',lambda b:struct.pack_into('<d',b,64+4*8,1.))
        reject('wrong_previous_w',lambda b:struct.pack_into('<d',b,64+(7+8*3132)*8,99.))
        reject('wrong_dual_u',lambda b:struct.pack_into('<d',b,64+(7+4*3132)*8,99.))
        reject('wrong_previous_u',lambda b:struct.pack_into('<d',b,64+(7+11*3132)*8,99.))
        reject('wrong_a',lambda b:struct.pack_into('<d',b,64+7*8,99.))
        reject('invalid_iteration',lambda b:struct.pack_into('<d',b,64+8,20001.))
    a.report.write_text(json.dumps(dict(passed=True,checks=checks,hardware_access=False),indent=2)+'\n','utf-8')
    print('STATE_AUDIT_NEGATIVE_PASS',len(checks))
if __name__=='__main__':main()
