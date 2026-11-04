"""Adapters for pinned historical measurements and captured RAW files."""
from pathlib import Path

from contracts import require, pinned, read_json, sha


def release_info(config, workspace):
    root = (Path(workspace)/config['release']).resolve()
    data = (root/'release_manifest.json').read_bytes()
    require(sha(data) == config['release_manifest_sha256'], 'RELEASE_HASH', 'Frozen release manifest changed')
    import json
    manifest = json.loads(data)
    require(manifest['software_revision'] == '20261017-p789-p1011-v1' and
            manifest['hardware_release'] == '20261016-rp2-p56-v1' and manifest['layout_version'] == 0x32525001,
            'RELEASE_VERSION', 'Unexpected complete algorithm release')
    return root,manifest


def release_file(root, manifest, name):
    record = dict(manifest['files'][name],path=name)
    return pinned(root,record)


def historical_case(config, workspace, case):
    root,manifest = release_info(config,workspace)
    import json
    rows = json.loads(release_file(root,manifest,'data/manifest.json'))
    matches = [r for r in rows if r['key']==case]
    require(len(matches)==1,'UNKNOWN_CASE','Unknown or ambiguous historical case')
    row = matches[0]
    def record(role):
        r = row[role]
        name = 'data/'+r['file']
        require(manifest['files'][name] == {k:r[k] for k in ('bytes','sha256')},'DATA_MANIFEST','Dataset/release manifests differ')
        return dict(path=str(root/name),bytes=r['bytes'],sha256=r['sha256'])
    desc = dict(schema_version=1,kind='PFY',sample_id=case,complete=True,origin='HISTORICAL_MATCHED_MEASUREMENT',
                file=record('measurement'),model=record('model'),model_id=row['model_id'],golden=record('golden'),capture={})
    return desc,root,row


def historical_raw(config, workspace, sample):
    import json
    catalog = json.loads(pinned(workspace,config['raw_catalog']))
    rows = [r for r in catalog['samples'] if r['sample_id']==sample]
    require(len(rows)==1,'UNKNOWN_SAMPLE','Unknown or ambiguous historical RAW')
    r = rows[0]
    def rec(role):
        v = r[role]
        return dict(path=str(Path(workspace)/v['workspace_path']),bytes=v['bytes'],sha256=v['sha256'])
    # Verify the original capture register and session records as well as RAW/sidecar.
    active = json.loads(pinned(workspace,rec('camera_registers')))
    state = json.loads(pinned(workspace,rec('session_record')))
    cap = r['capture']
    fmt = dict(r['format'],valid_bits=12,roi_sensor_offset=r['geometry']['roi_sensor_offset'],
               binning=cap['binning'],orientation=r['geometry']['orientation_transform'])
    return dict(schema_version=1,kind='RAW',sample_id=sample,complete=True,origin=r['source_type'],file=rec('raw'),
        metadata=rec('metadata'),format=fmt,calibration=None,model_id=None,model=None,
        capture=dict(session_id=cap['boot_id'],original_sensor_frame_id=cap['frame_number'],original_block_id=cap['block_id'],
                     camera_timestamp_ticks=cap['camera_timestamp_ticks'],camera_tick_frequency_hz=cap['timestamp_tick_frequency_hz'],
                     exposure_us=cap['exposure_us'],gain_raw=cap['gain_register_raw'],gain_units=cap['gain_physical_units']),
        geometry=r['geometry'],source_records_verified=True,warnings=r['warnings'])
