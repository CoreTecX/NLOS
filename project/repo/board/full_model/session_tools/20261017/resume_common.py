"""Control tools bound to the explicitly observed afternoon boot."""
from pathlib import Path
import base64,json,zlib
import jtag_transfer as transport

ROOT=Path(__file__).resolve().parent
REMOTE=transport.REMOTE
BOOT='82d3d957-6382-431f-9c6c-26e3b9f74665'
_serial=transport.serial

def serial(code,name,timeout=50):
    guarded="from pathlib import Path\nif Path('/proc/sys/kernel/random/boot_id').read_text().strip()!="+repr(BOOT)+":raise RuntimeError('Afternoon boot changed; inspect before continuing')\n"+code
    if len(guarded.encode())>2300:
        guarded='import base64,zlib\nexec(zlib.decompress(base64.b64decode('+repr(base64.b64encode(zlib.compress(guarded.encode(),9)).decode())+')))\n'
    return _serial(guarded,name,timeout)

def bind():
    transport.BOOT=BOOT
    transport.serial=serial

if __name__=='__main__':
    import runpy,sys
    bind()
    tool=ROOT/sys.argv[1]
    if tool.name not in ('jtag_download.py','board_control.py','upload_serial_file.py','capture_display.py'):
        raise ValueError('Unsupported afternoon tool')
    sys.argv=sys.argv[1:]
    runpy.run_path(str(tool),run_name='__main__')
