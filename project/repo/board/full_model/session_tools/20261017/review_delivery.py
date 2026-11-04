"""Verify final artifact coverage, provenance, scientific arrays and screen history."""
from pathlib import Path
import hashlib,json
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parent

def main():
    dest=ROOT/'delivery_final'
    s=json.loads((dest/'summary.json').read_text('utf8'))
    audit=json.loads((dest/'verification/dataset97_acceptance.json').read_text('utf8'))
    original=json.loads((dest/'verification/original15_board_acceptance.json').read_text('utf8'))
    manifest=json.loads((dest/'images/dataset97_pl_01/manifest.json').read_text('utf8'))
    rows={x['key']:x for x in audit['cases']}
    if len(rows)!=97 or not audit['passed'] or not all(x['passed'] for x in rows.values()):raise RuntimeError('97-case audit is incomplete')
    if not original['passed'] or len(original['cases'])!=15 or original['missing']:raise RuntimeError('Original 15-case release acceptance incomplete')
    if s['batch']['accepted_cases']!=97 or s['batch']['completed']!=97 or s['batch']['original15_accepted']!=15:raise RuntimeError('Summary count mismatch')
    if manifest['count']!=97 or {x['key'] for x in manifest['cases']}!=set(rows):raise RuntimeError('Image coverage mismatch')
    raw=dest/'results/dataset97_pl_01';images=dest/'images/dataset97_pl_01'
    for row in manifest['cases']:
        key=row['key'];data=(raw/(key+'.pfo')).read_bytes()
        if hashlib.sha256(data).hexdigest()!=row['source_pfo_sha256']:raise RuntimeError('Image source identity mismatch')
        expected=np.frombuffer(data[64:],dtype='<f8').reshape(3,29,36,3)
        with np.load(images/key/'reconstruction_fp64.npz') as archive:
            for i,name in enumerate(('h','normalized','physical')):
                if not np.array_equal(archive[name],expected[i]):raise RuntimeError('FP64 image export differs: '+key)
        for name,size in (('reconstruction_linear.png',(36,29)),('reconstruction_srgb.png',(36,29)),('preview_10x.png',(360,290))):
            with Image.open(images/key/name) as im:
                if im.size!=size or im.mode!='RGB':raise RuntimeError('Image shape or mode mismatch')
                im.verify()
    final=dest/'evidence/final_snapshot/content'
    env=json.loads((final/'final_board_evidence.json').read_text('utf8'))
    if not env['original_config_unchanged'] or env['active']:raise RuntimeError('Unexpected final configuration or active algorithm')
    history=[json.loads(x) for x in (final/'display/live_history.jsonl').read_text('utf8').splitlines()]
    resumed=[x for x in history if x.get('boot_id')==s['boot_ids'][1]]
    numbers=[x['number'] for x in resumed]
    if sorted(set(numbers))!=list(range(52,98)) or numbers!=sorted(numbers):raise RuntimeError('Resume display skipped or reordered a case')
    for row in resumed:
        if hashlib.sha256((raw/(row['case']+'.pfo')).read_bytes()).hexdigest()!=row['source_sha256']:raise RuntimeError('Display source identity mismatch')
    state=json.loads((final/'display/live_state.json').read_text('utf8'))
    if state['number']!=97 or state['independent_acceptance'] is not True:raise RuntimeError('Final screen is not independently accepted')
    source=ROOT/'resume_capture_final_display.log'
    capture=json.loads(source.read_text('utf8'));shot=Path(capture['file'])
    if not (dest/'display'/shot.name).is_file() or capture['state']['framebuffer_sha256']!=state['framebuffer_sha256']:raise RuntimeError('Final screenshot differs from display state')
    report=dict(passed=True,cases=97,original15_passed=True,fp64_arrays_exact=True,image_dimensions_verified=True,afternoon_display_numbers=sorted(set(numbers)),boot_ids=s['boot_ids'],final_screenshot=shot.name,all_board_gates_passed=False,maximum_relative_normalized=max(x['numeric']['relative_normalized'] for x in rows.values()))
    with (ROOT/'delivery_review.json').open('x',encoding='utf8') as f:json.dump(report,f,indent=2)
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
