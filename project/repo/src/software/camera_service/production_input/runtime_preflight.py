"""Read-only runtime preflight; never open camera, serial, FPGA or display devices."""
import argparse
import importlib
import json
import platform
import sys
from contracts import read_json
from native_backend import validate_runtime


def check(config):
    report = dict(passed=False, python=sys.version, machine=platform.machine(), system=platform.system(),
                  hardware_accessed=False, dependencies={}, errors=[])
    try:
        report['target'], report['backend'] = validate_runtime(config)
    except Exception as error:
        report['errors'].append(str(error))
    for name in ('numpy', 'scipy', 'PIL'):
        try:
            module = importlib.import_module(name)
            report['dependencies'][name] = getattr(module, '__version__', 'unknown')
        except Exception as error:
            report['errors'].append(name+': '+str(error))
    try:
        from scipy.fft import dctn, idctn
        import numpy as np
        value=np.arange(12,dtype=np.float64).reshape(3,4)
        report['dct_roundtrip'] = bool(np.allclose(value,idctn(dctn(value,norm='ortho'),norm='ortho'),rtol=0,atol=1e-12))
        if not report['dct_roundtrip']:
            report['errors'].append('DCT roundtrip failed')
    except Exception as error:
        report['errors'].append('DCT: '+str(error))
    report['passed'] = not report['errors']
    return report

if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',required=True)
    report=check(read_json(p.parse_args().config));print(json.dumps(report,ensure_ascii=False,indent=2))
    raise SystemExit(0 if report['passed'] else 2)
