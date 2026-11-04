"""Generate a pinned synthetic RAW-motion manifest for the offline runner."""
from motion_fixture import *
from contracts import decode_pfy
from raw_measurement import convert_raw


def make_demo(root):
    root=Path(root);raw,desc,cal,anchor,contract=create_fixture(root)
    source=FileSource();f,d=frame_for(root,source,desc,raw,0)
    rgb=decode_pfy(convert_raw(f,d,cal,root,D11['model']['sha256'])[0])[2]
    mask=np.zeros((378,378),bool);mask[15:42,15:-15]=True;mask[-42:-15,15:-15]=True;mask[15:-15,15:42]=True;mask[15:-15,-42:-15]=True
    np.savez(root/'fast_reference.npz',reference=rgb,stable_mask=STABLE)
    np.savez(root/'motion_anchor.npz',raw=raw,stable_mask=mask)
    ctx=context(session='synthetic_motion').changed(calibration_id=cal['calibration_id'],roi_id=cal['roi_id'],operator_id=cal['operator_id'])
    fields=ctx.data
    for k in ('compat_generation','model_epoch','geometry_epoch','radiometry_epoch'):fields[k]=int(fields[k])
    binding=dict(context=ctx.data,reference_arrays=record(root/'fast_reference.npz'),fast_policy=dict(FAST,radiometric_normalizer=1000,ttl_ms=5000),
        motion=dict(calibration=record(root/'calibration.json'),contract=record(root/'motion_contract.json'),
            anchor_arrays=record(root/'motion_anchor.npz'),policy=dict(tracking_low=450,tracking_high=1450,minimum_inliers=12)))
    frames=[]
    for i in range(4):
        _,d=frame_for(root,source,desc,generate(similarity(i,-i,.1*i)),i+1)
        write_json(root/f'input_{i}.json',d)
        frames.append(dict(at_ms=i*1000,sample_time_ns=str((i+1)*100_000_000),descriptor=record(root/f'input_{i}.json')))
    spec=dict(schema_version=1,scope='SYNTHETIC_TEST_ONLY',context=fields,bindings=[binding],models=[D11],
              scheduler_policy=dict(POLICY,capture_gap_ms=2000),frames=frames)
    write_json(root/'sequence.json',spec)
    # Display-only views are derived from RAW, never used as solver input.
    from PIL import Image
    for name,a in [('reference',raw),('moved',generate(similarity(10,-8,2,1.01)))]:
        Image.fromarray(np.uint8(np.clip((a.astype(float)-450)/1000,0,1)*255)).save(root/(name+'_display.png'))
    return root/'sequence.json'


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    print(make_demo(a.output))
