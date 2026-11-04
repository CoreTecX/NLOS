"""Prepare exact workspace updates, then publish only the reviewed plan."""
from pathlib import Path
import argparse,csv,hashlib,json,os,shutil
ROOT=Path(__file__).resolve().parent
WORKSPACE=Path(r'D:\学习\26秋嵌入式\PenumbraSight工作区')
REPO=WORKSPACE/'02_代码仓库/penumbrasight'
REL='03_任务产出/Board_Steps_5_12_20261017'
MIRROR='02_代码仓库/penumbrasight/sim/results/board_steps_5_12_20261017'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def prepare(delivery):
    stage=ROOT/(delivery.name+'_workspace_update');stage.mkdir(exist_ok=False)
    s=json.loads((delivery/'summary.json').read_text('utf8'));batch=s['batch'];passed=bool(batch['numerically_accepted'])
    detail='97/97例实板重建、数值与PFS双重验收通过，含原15例及85例论文测量' if passed else f"97例批处理中，当前记录已运行{batch['completed']}/97，尚未全量验收"
    statement='最新算法已在实板CPU、FPGA同步及重叠模式跑通d11_rgb并双重验收；'+detail+'。'
    gate='B02通过；B03仅CPU1/15；'+('B04重叠15/15通过；' if passed else 'B04部分完成；')+'B01有未闭环提示，B05–B07部分记录，B08待实测。'
    replacements={
        '已完成离线开发和交付，尚未上板。':statement,
        '离线交付完成，最新版本尚未上板。':statement,
        '当前完整算法离线交付完成，尚未上板。':statement,
        '完整1044维FP64，当前发布仍未上板。':'完整1044维FP64。'+statement,
        'B01–B08 均为 PENDING_BOARD。':gate,
        'B01–B08均待板。':gate,
        'B01–B08 均待板。':gate,
        'B01–B08全部待板':gate.rstrip('。'),
        'ARM/DDR/PL实机与B01–B08均待板。':statement+' '+gate,
        '第5–14步未执行。':'第5–12步已按用户授权推进，最新执行证据见CURRENT_BOARD_RUN.md；第13–14步不作完成声明。',
        '仅电脑端准备，不改变实板待验证状态。':'此段为第1–4步历史范围；后续实板证据见CURRENT_BOARD_RUN.md。',
        '最新完整算法的板端运行尚未验收；真实相机到实时显示的整机线路尚未全部接通。':statement+'真实相机到实时显示的整机线路尚未全部接通。',
        '主机协议验证及匹配的RTL/布局布线证据；实际ARM/DDR/PL待板':'已新增真实ARM/DDR/PL执行及独立验收；详见本次实板报告',
        '整包只读自检；两套包均未上板，性能排序待实测':'两套包已部署并通过板端自检；150MHz性能包已实跑，100MHz回退实跑仍待验证',
        '| B01–B08 板端验收 | PENDING_BOARD |':'| B01–B08 板端验收 | 部分完成，见本次实板任务表 |',
        'B01–B08待板；错误后复位重载':'B01–B08按本次实板任务表记录；错误后复位重载',
        'ARM/DDR/缓存属性/实际恢复待板':'ARM/DDR正常路径已实跑；故障恢复仍待验证',
        '实际镜像与时钟待板':'匹配镜像及150MHz正常路径已实跑',
        '硬件门禁保持待板':'实板门禁按本次B01–B08部分完成状态记录'
    }
    replacements.update({
        '实时采集/选模/UI集成及实板验收仍待执行。':'实时采集/选模/UI整机集成仍待执行；文件批量实板与逐例屏显证据见当前运行报告。',
        'I3待集成；当前show_framebuffer展示工具不等于实时UI闭环':'文件批量逐例屏显已完成；相机连续帧到整机实时UI仍待集成',
        '完整文件输入到输出链已经离线验证；相机RAW、自动定位选模和实时UI连接仍待集成，B01–B08实际运行待板。':'完整文件输入到输出链已新增真实板端验收与逐例屏显；相机RAW、自动定位选模和整机实时UI仍待集成。'+gate,
        '完成离线验证。最新完整算法的板端运行尚未验收；':'完成离线验证并新增实板验收。'+statement,
        '当前软件/硬件未上板':'当前文件算法已有实板证据，完整板端门禁仍部分待验证',
        'P11.4待板':'P11.4已部分实测，剩余门禁见B01–B08',
        'B01–B08实际运行待板':'B01–B08已有部分实测，详见当前运行报告',
        '离线交付完成、实板待验':'离线交付完成、实板部分门禁已验收',
        'I1–I3 是连接边界清单，本次只登记状态，没有实施。':'I1–I3 是整机连接边界清单；本次新增文件批量逐例屏显，相机/自动选模/连续帧整机接入仍未实施。'
    })
    catalog=json.loads((REPO/'configs/workspace_modules.json').read_text('utf8'))
    docs=['readme.md','00_总索引.md','CURRENT_BOARD_RELEASE.md','00_总览/最新成果与任务状态.md','01_分析报告/00_当前进程与任务总表.md','02_代码仓库/penumbrasight/readme.md','02_代码仓库/penumbrasight/docs/current_full_model.md','02_代码仓库/penumbrasight/docs/module_status.md','02_代码仓库/penumbrasight/docs/system_chain_status.md','02_代码仓库/penumbrasight/docs/workspace_maintenance.md','02_代码仓库/penumbrasight/board/full_model/readme.md']
    docs=list(dict.fromkeys(docs+['02_代码仓库/penumbrasight/'+m['readme'] for m in catalog['modules']]))
    changes=[]
    def add(rel,new):
        target=WORKSPACE/rel;p=stage/'files'/rel;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(new,encoding='utf8')
        changes.append(dict(path=rel,old_sha256=sha(target) if target.exists() else None,new_sha256=sha(p),staged=str(p)))
    for rel in docs:
        path=WORKSPACE/rel;text=path.read_text('utf8')
        for old,new in replacements.items():text=text.replace(old,new)
        link=os.path.relpath(WORKSPACE/'CURRENT_BOARD_RUN.md',path.parent).replace('\\','/')
        banner='> 2026-10-17 最新实板状态：'+statement+' [运行证据与图片入口]('+link+')。\n\n'
        first,sep,rest=text.partition('\n');text=first+'\n\n'+banner+rest.lstrip('\n')
        if rel.endswith('docs/system_chain_status.md'):
            text=text.replace('D -. 待接入连续显示与状态管理 .-> G[实时UI]','D --> H[文件批量逐例显示：已实板验证]\n    D -. 待接入相机连续帧与状态管理 .-> G[整机实时UI]')
            text=text.replace('| I3 重建结果到实时UI | 待集成 |','| I3 重建结果到实时UI | 文件批量屏显已完成；整机接入仍待集成 |')
        if rel.endswith('docs/workspace_maintenance.md'):
            text+='\n实板证据单独登记在`configs/current_board_run.json`。原模块登记的`physical_board_test=false`属于离线登记核验范围，不能用来否认新增的实板记录；冻结发布manifest不回写。\n'
        add(rel,text)
    latest=dict(updated=s['updated'],physical_board_test=True,software_revision=s['software_revision'],hardware_release=s['hardware_release'],profile=s['profile'],boot_ids=s['boot_ids'],completed_cases=batch['completed'],independently_accepted_cases=batch.get('accepted_cases',0),all_board_gates_passed=False,board_acceptance=s['board_acceptance'],report='sim/results/board_steps_5_12_20261017/执行报告.md',summary='sim/results/board_steps_5_12_20261017/summary.json')
    add('02_代码仓库/penumbrasight/configs/current_board_run.json',json.dumps(latest,ensure_ascii=False,indent=2)+'\n')
    catalog['physical_board_test_scope']='离线模块登记核验；当前实板证据另见current_board_run，冻结发布不回写。'
    catalog['current_board_run']='configs/current_board_run.json'
    affected={'MOD02','MOD09','MOD10','MOD13','MOD14','MOD22','MOD23','MOD25','MOD26','MOD27','MOD29','MOD30','MOD33','MOD34','MOD36','MOD38'}
    for row in catalog['modules']:
        if row['id'] in affected:
            row['board_evidence']='sim/results/board_steps_5_12_20261017/执行报告.md'
            row['board_evidence_scope']='项目文件批量运行已新增实板证据；不表示本模块所有接口、异常恢复或相机整机集成都已验收。'
        for old,new in replacements.items():
            row['boundary']=row['boundary'].replace(old,new)
    add('02_代码仓库/penumbrasight/configs/workspace_modules.json',json.dumps(catalog,ensure_ascii=False,indent=2)+'\n')
    current=f'''# 当前实板运行状态

更新：{s['updated']}。{statement}

软件20261017-p789-p1011-v1；硬件20261016-rp2-p56-v1；150MHz/burst128；完整1044维FP64。

- ARM NEON3220项自检全部通过。
- d11_rgb纯求解：CPU 376.377秒、FPGA同步36.568秒、重叠28.858秒；各一次，CPU约525MHz，不代表最高性能。
- 开发板DP屏幕逐例更新；下午从第52例恢复并显示第53–97例，最终标注97例已验收，帧缓冲回读核验通过。上午自动展示从第24例开始。
- 上午52例、下午45例分目录保留；午间用户要求的安全关机与两次boot ID均有记录，合并逐项校验身份与SHA256。
- {gate}
- 47份论文测量缺匹配模型/标定、1例工程输入独立参考不收敛，仍未进入合格批量。
- USB开机过流提示及FAT未正常卸载提示原因尚未闭环；相机、自动选模及实时UI仍待集成。

[执行报告]({REL}/执行报告.md) · [原始结果]({REL}/results/) · [验收明细]({REL}/verification/) · [图片目录]({REL}/images/dataset97_pl_01/index.html) · [显示屏画面]({REL}/display/) · [B01–B08任务表]({REL}/board_acceptance.csv)

本次新增证据独立归档，不改写P7–P11和第1–4步冻结发布；历史报告中的“未上板”是其交付时状态。板端结果目录：/home/root/ps_runs_20261017/steps5_12_01。
'''
    add('CURRENT_BOARD_RUN.md',current)
    dataset=f'''# 当前上板数据状态

更新：{s['updated']}。第1–4步离线准备保留原包，后续实板任务已推进。

{detail}。97例由85例论文测量和12例工程输入组成。

47份论文测量仍缺匹配模型/标定，1例已适配工程压力输入的独立参考不收敛；未把这些文件记为已重建。GT、论文版面截图及已有结果图不作为墙面测量输入。

[当前实板状态](CURRENT_BOARD_RUN.md) · [实板报告]({REL}/执行报告.md) · [图片目录]({REL}/images/dataset97_pl_01/index.html) · [原准备报告](03_任务产出/Board_Steps_1_4_20261017/执行报告.md) · [论文阻塞清单](03_任务产出/Board_Steps_1_4_20261017/blocked_paper.csv)

数据包和两套原发布保持冻结哈希，所有板端运行结果另存新目录。
'''
    add('CURRENT_BOARD_DATASET.md',dataset)
    board_csv=REPO/'board/full_model/board_acceptance.csv'
    with board_csv.open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
    states={x['id']:x for x in s['board_acceptance']}
    import io
    buf=io.StringIO(newline='');w=csv.DictWriter(buf,fieldnames=['id','task','criterion','status','result','evidence_path']);w.writeheader()
    for row in rows:
        state=states[row['id']];row.update(status=state['status'],result=state['result'],evidence_path='sim/results/board_steps_5_12_20261017/执行报告.md');w.writerow(row)
    add('02_代码仓库/penumbrasight/board/full_model/board_acceptance.csv',buf.getvalue())
    add('02_代码仓库/penumbrasight/docs/board_run_20261017.md',current.replace(REL,'../sim/results/board_steps_5_12_20261017').replace('](CURRENT_BOARD_RUN.md)','](../../../CURRENT_BOARD_RUN.md)'))
    add('01_分析报告/实板算法运行_20261017.md',current.replace(REL,'../'+REL))
    plan=dict(delivery=str(delivery),destinations=[REL,MIRROR],files=changes,tools_destination='02_代码仓库/penumbrasight/board/full_model/session_tools/20261017',backup='99_临时工作区/board_steps_5_12_20261017/previous_current_documents')
    (stage/'plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2),encoding='utf8')
    print('PREPARED',str(stage/'plan.json'),len(changes),flush=True)

def publish(planpath):
    plan=json.loads(planpath.read_text('utf8'));delivery=Path(plan['delivery'])
    for row in plan['files']:
        target=WORKSPACE/row['path'];current=sha(target) if target.exists() else None
        if current!=row['old_sha256']:raise RuntimeError('Workspace changed since plan: '+row['path'])
        if sha(Path(row['staged']))!=row['new_sha256']:raise RuntimeError('Staged file changed')
    for rel in plan['destinations']+[plan['tools_destination']]:
        if (WORKSPACE/rel).exists():raise RuntimeError('Refusing to overwrite existing delivery: '+rel)
    for rel in plan['destinations']:shutil.copytree(delivery,WORKSPACE/rel)
    shutil.copytree(delivery/'tools',WORKSPACE/plan['tools_destination'])
    for row in plan['files']:
        target=WORKSPACE/row['path'];target.parent.mkdir(parents=True,exist_ok=True)
        if target.exists():
            backup=WORKSPACE/plan['backup']/row['path'];backup.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(target,backup)
        shutil.copy2(row['staged'],target)
    checks=[]
    for p in delivery.rglob('*'):
        if p.is_file():
            rel=p.relative_to(delivery);expected=sha(p)
            for dest in plan['destinations']:
                actual=WORKSPACE/dest/rel
                if actual.stat().st_size!=p.stat().st_size or sha(actual)!=expected:raise RuntimeError('Mirror mismatch '+str(actual))
            checks.append(dict(path=str(rel),bytes=p.stat().st_size,sha256=expected))
    for row in plan['files']:
        if sha(WORKSPACE/row['path'])!=row['new_sha256']:raise RuntimeError('Status publication mismatch')
    receipt=dict(passed=True,files_per_mirror=len(checks),mirrors=plan['destinations'],updated_documents=len(plan['files']),checks=checks)
    (ROOT/'publication_receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2),encoding='utf8')
    for dest in plan['destinations']:shutil.copy2(ROOT/'publication_receipt.json',WORKSPACE/dest/'publication_receipt.json')
    print('PUBLISHED_AND_VERIFIED',len(checks),'files per mirror',flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','publish']);p.add_argument('path',type=Path);a=p.parse_args()
    if a.action=='prepare':prepare(a.path)
    else:publish(a.path)
