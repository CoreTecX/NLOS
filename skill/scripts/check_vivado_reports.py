"""Check a routed-audit JSON; missing fields fail closed."""
import argparse,json,sys
p=argparse.ArgumentParser(description=__doc__);p.add_argument('audit');a=p.parse_args();v=json.load(open(a.audit,encoding='utf8'))
ok=all(k in v for k in ['wns_ns','whs_ns','black_boxes','drc_errors']) and v.get('wns_ns',-1)>=0 and v.get('whs_ns',-1)>=0 and v.get('black_boxes',1)==0 and v.get('drc_errors',1)==0
print('PASS: routed timing/DRC' if ok else 'FAIL: timing, DRC, black boxes or missing evidence');sys.exit(0 if ok else 1)
