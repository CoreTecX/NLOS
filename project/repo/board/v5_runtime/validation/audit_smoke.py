"""Independent FP64 numerical/state acceptance for a downloaded D11 run."""
from pathlib import Path
import argparse, hashlib, json, sys

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--release', type=Path, required=True)
    p.add_argument('--results', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    sys.path.insert(0, str(a.release / 'scripts'))
    import accept_board, accept_state
    row = next(x for x in json.loads((a.release / 'data/manifest.json').read_text()) if x['key'] == 'D11_rgb')
    prefix = a.results / 'D11_rgb'
    numerical = accept_board.evaluate(row, prefix.with_suffix('.pfo'))
    state = accept_state.evaluate(a.release / 'data' / row['model']['file'], a.release / 'data' / row['measurement']['file'], prefix)
    result = {'passed': numerical['passed'] and state['passed'], 'numerical': numerical, 'independent_state': state,
              'artifacts': {suffix: hashlib.sha256(prefix.with_suffix(suffix).read_bytes()).hexdigest() for suffix in ['.pfo', '.pfs', '.ppm']}}
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if not result['passed']:
        raise SystemExit(1)

if __name__ == '__main__':
    main()
