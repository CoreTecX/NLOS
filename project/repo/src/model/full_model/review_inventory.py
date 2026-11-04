"""Account for prepared copies, numeric references and legacy inputs without rerunning them."""
from pathlib import Path
import argparse,collections,hashlib,json
import numpy as np
from scipy.io import loadmat
from prepare_dataset import decode_measurement,dump,csv_write

def array_hash(a):
    a=np.asarray(a,dtype='<f8',order='C')
    return hashlib.sha256(str(a.shape).encode()+a.tobytes()).hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();out=a.output
    assets=json.loads((out/'inventory/assets.json').read_text('utf8'));m=json.loads((out/'dataset/dataset_manifest.json').read_text('utf8'))
    hashes={}
    for case in m['cases']:
        _,y,sc=decode_measurement(out/'dataset'/case['measurement']['file'])
        hashes.setdefault(array_hash(y),[]).append(case['key'])
    audit=out/'inventory/numeric_provenance.json'
    provenance={r['path']:r for r in json.loads(audit.read_text('utf8'))} if audit.exists() else {}
    dispositions=[]
    for asset in assets:
        path=Path(asset['path']);role=asset['role'];r=dict(path=asset['path'],sha256=asset['sha256'],role=role)
        if role in ['raw_measurement','auxiliary_localization_or_model'] and asset['scope']=='paper':r['disposition']='SEE_PAPER_CASES'
        elif role=='prepared_measurement_candidate':r['disposition']='SEE_PROJECT_PFY_SOURCES'
        elif role=='raw_or_prepared_candidate':
            variables={v['name'] for v in asset.get('variables',[])}
            selected=variables&{'image','test_image1','measurements'}
            d=loadmat(path,variable_names=sorted(selected));matches=[];unknown=[]
            for name in selected:
                arr=d[name]
                values=arr.ravel().tolist() if arr.dtype==object else [arr]
                for idx,value in enumerate(values):
                    if np.asarray(value).shape==(126,126,3):
                        keys=hashes.get(array_hash(value),[])
                        if keys:matches.append(dict(field=name,index=idx,cases=keys))
                        else:unknown.append(dict(field=name,index=idx,shape=list(np.shape(value))))
                    else:unknown.append(dict(field=name,index=idx,shape=list(np.shape(value))))
            r.update(disposition='PREPARED_MEASUREMENT_CONTENT_DUPLICATE' if matches and not unknown else 'HISTORICAL_OR_UNBOUND_MEASUREMENT',matches=matches,unbound=unknown,
                     note='Equal measurement arrays do not erase historical model/calibration differences; current cases use only pinned matching models')
        elif role=='legacy_numeric_array_requires_geometry_and_scale_contract':
            arr=np.load(path,allow_pickle=False);keys=hashes.get(array_hash(arr),[]) if arr.shape==(126,126,3) else []
            if keys:r.update(disposition='MEASUREMENT_CONTENT_DUPLICATE',matches=keys)
            elif 'gt' in path.stem:r.update(disposition='GROUND_TRUTH_REFERENCE_ONLY',reason='Not an observed wall measurement; not used by inference')
            elif 'meas' in path.stem:r.update(disposition='LEGACY_SYNTHETIC_OR_UNBOUND_MEASUREMENT',reason='Different shape or unproven geometry/scales; retained, not padded or guessed')
            else:r['disposition']='MODEL_DIAGNOSTIC_OR_EXISTING_RECONSTRUCTION'
        elif role=='legacy_quantized_input_requires_original_source':r.update(disposition='LEGACY_PSF_RETAINED',reason='PSF is not PFY; use original raw source association from historical matlab_input_inventory.json; do not upconvert quantized data silently')
        elif role=='display_reference_or_document_image_not_sensor_input':r.update(disposition='DISPLAY_OR_DOCUMENT_ASSET',reason='Existing reconstruction, plot, screenshot or document illustration; retained for comparison, not fed back as a new sensor measurement')
        elif role=='derived_model_reference_or_solver_state':
            candidates=set(asset.get('array_keys',[]))&{'image','measurement','test_image1','raw','y'}
            r.update(disposition='MODEL_REFERENCE_OR_EXPERIMENT_STATE',array_keys=asset.get('array_keys',[]))
            if candidates:r.update(disposition='NUMERIC_INPUT_REQUIRES_PROVENANCE_REVIEW',candidate_fields=sorted(candidates))
            fixture_names={'meas_rtl_t2_seam','mush_single_frame_diffs','profile_b_96x96','v1_block_small','v2_frame_full','v3_accum_frames','v4_edge_cases','x6_x7_bitdepth'}
            if path.stem in fixture_names:
                shapes={};metadata={}
                with np.load(path,allow_pickle=False) as z:
                    for field in z.files:
                        try:
                            arr=z[field];shapes[field]=list(arr.shape)
                            if field in {'source','meta_source','meta_note','meta_profile','meta_region'}:metadata[field]=str(arr.tolist())
                        except ValueError:shapes[field]='Object metadata not unpickled'
                r.update(disposition='LEGACY_PREPROCESSING_TEST_VECTOR',array_shapes=shapes,metadata=metadata,reason='Historical block/bit-depth/accumulation/difference/RTL fixture, including cropped or quantized data. Not an independently calibrated full-model input. Original canonical raw sources are handled through paper_cases; preserve the fixture without padding, inverse-differencing or guessed scales.')
        else:r['disposition']='AUXILIARY_OR_MODEL_ASSET'
        if r['path'] in provenance:
            r.update(provenance[r['path']]);r['provenance_report']='inventory/numeric_provenance.json'
        dispositions.append(r)
    dump(out/'inventory/dispositions.json',dispositions)
    csv_write(out/'inventory/dispositions.csv',dispositions,['path','role','disposition','matches','unbound','reason','sha256'])
    summary=dict(counts=dict(collections.Counter(r['disposition'] for r in dispositions)),
                 numeric_review=[r for r in dispositions if r['disposition']=='NUMERIC_INPUT_REQUIRES_PROVENANCE_REVIEW'],
                 unbound_prepared=[r for r in dispositions if r['disposition']=='HISTORICAL_OR_UNBOUND_MEASUREMENT'])
    dump(out/'inventory/disposition_summary.json',summary)
    print(json.dumps(dict(counts=summary['counts'],numeric_review=len(summary['numeric_review']),unbound_prepared=len(summary['unbound_prepared'])),ensure_ascii=False),flush=True)

if __name__=='__main__':main()
