"""Check portable lowercase ASCII names; README.md is the documented exception."""
from pathlib import Path
import argparse,re,sys
p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);a=p.parse_args()
bad=[str(f.relative_to(a.root)) for f in a.root.rglob('*') if '.git' not in f.parts and f.name!='README.md' and not re.fullmatch(r'[a-z0-9_.-]+',f.name)]
print('\n'.join(bad) if bad else 'PASS: portable names');sys.exit(bool(bad))
