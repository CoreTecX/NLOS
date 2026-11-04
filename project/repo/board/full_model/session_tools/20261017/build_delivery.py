"""Assemble reviewable board evidence without writing to the formal workspace."""
from pathlib import Path
import argparse,csv,hashlib,json,shutil,datetime
ROOT=Path(__file__).resolve().parent
WORKSPACE=Path(r'D:\学习\26秋嵌入式\PenumbraSight工作区')
PREP=WORKSPACE/'03_任务产出/Board_Steps_1_4_20261017'

def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()

def copy(source,dest):
    dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest)

def main():
    p=argparse.ArgumentParser();p.add_argument('--name',default='delivery_final');a=p.parse_args()
    dest=ROOT/a.name;dest.mkdir(exist_ok=False)
    summary=dict(updated=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),software_revision='20261017-p789-p1011-v1',hardware_release='20261016-rp2-p56-v1',profile='p56_150_b128',boot_ids=['de4ce291-1757-4556-aa85-e201e3d06f51','82d3d957-6382-431f-9c6c-26e3b9f74665'],dct=json.loads((ROOT/'dct_selftest_verified.json').read_text('utf8')),single_cases=[],batch=None,paper_blocked=47,engineering_reference_failed=1)
    exports={'cpu_d11_v2':'cpu_d11_01','pl_sync_d11_01':'pl_sync_d11_01','pl_overlap_d11_01':'pl_overlap_d11_01'}
    for export,name in exports.items():
        local=ROOT/'downloads'/export
        if not(local/'verified.json').exists():continue
        content=local/'content';shutil.copytree(content/'results'/name,dest/'results'/name)
        for file in content.rglob('*'):
            if file.is_file() and 'results' not in file.relative_to(content).parts:copy(file,dest/'evidence/board'/export/file.relative_to(content))
        for file in ['manifest.json','verified.json']:copy(local/file,dest/'evidence/transport'/export/file)
        if name!='dataset97_pl_01':
            acceptance=ROOT/(name.replace('_01','')+'_acceptance.json')
            result=json.loads((content/'results'/name/'D11_rgb.json').read_text('utf8'))
            audit=json.loads(acceptance.read_text('utf8'));copy(acceptance,dest/'verification'/acceptance.name)
            summary['single_cases'].append(dict(name=name,iterations=result['iterations'],seconds_solver=result['seconds_solver'],passed=audit['passed'],relative_normalized=audit['cases'][0]['numeric']['relative_normalized']))
    if (ROOT/'batch_verified.json').exists():
        merged=ROOT/'merged/dataset97_pl_01'
        shutil.copytree(merged,dest/'results/dataset97_pl_01')
        for export in ('lunch52_snapshot','dataset45_pl_resume_01'):
            source=ROOT/'downloads'/export
            if json.loads((source/'verified.json').read_text('utf8'))['passed'] is not True:raise RuntimeError('Unverified source export')
            shutil.copytree(source/'content',dest/'evidence/raw_sessions'/export)
            for name in ('manifest.json','verified.json'):copy(source/name,dest/'evidence/transport'/export/name)
        for name in ('lunch52_acceptance.json','resume45_acceptance.json','original15_board_acceptance.json'):
            copy(ROOT/name,dest/'verification'/name)
    if (ROOT/'batch_progress.json').exists():
        progress=json.loads((ROOT/'batch_progress.json').read_text('utf8'))
        summary['batch']=dict(state=progress['state'],completed=len(progress.get('suite',{}).get('cases',[])),total=97,exit_code=progress.get('exit_code'),numerically_accepted=False,completed_before_pause=52,completed_after_resume=progress.get('completed_after_resume',0),user_requested_power_cycle=True)
    if (ROOT/'dataset97_acceptance.json').exists():
        audit=json.loads((ROOT/'dataset97_acceptance.json').read_text('utf8'));copy(ROOT/'dataset97_acceptance.json',dest/'verification/dataset97_acceptance.json')
        summary['batch']['numerically_accepted']=audit['passed'];summary['batch']['accepted_cases']=sum(x['passed'] for x in audit['cases'])
    if (ROOT/'images').exists():shutil.copytree(ROOT/'images',dest/'images')
    if (ROOT/'display').exists():shutil.copytree(ROOT/'display',dest/'display')
    if (ROOT/'downloads/final_evidence/verified.json').exists():
        shutil.copytree(ROOT/'downloads/final_evidence',dest/'evidence/final_snapshot',ignore=shutil.ignore_patterns('chunk.bin','read_chunk.tcl','results.tar.gz'))
    tool_names={'serial_command.ps1','board_control.py','board_unpack.py','board_job.py','board_export.py','jtag_transfer.py','jtag_download.py','prepare_transfer.py','acquire_transfer_lease.py','prepare_board_session.py','observe_board.py','wait_job.py','accept_original.py','render_results.py','finish_batch.py','prepare_display.py','show_display.py','prepare_live_display.py','upload_serial_file.py','live_display.py','capture_display.py','finalize_display.py','final_board_evidence.py','complete_session.py','build_delivery.py','publish_delivery.py'}
    tool_names.update({'resume_common.py','resume_verify_board.py','resume_prepare_results.py','resume_start_batch.py','resume_finish_batch.py','resume_finalize.py','live_display_resume_v2.py','review_delivery.py'})
    for file in ROOT.iterdir():
        if not file.is_file():continue
        if file.name in tool_names:copy(file,dest/'tools'/file.name)
        elif file.suffix in ['.json','.jsonl','.log']:copy(file,dest/'evidence/local'/file.name)
    copy(ROOT/'午间暂停与下午续跑.md',dest/'evidence/午间暂停与下午续跑.md')
    copy(ROOT/'下午续跑工具调整.md',dest/'evidence/下午续跑工具调整.md')
    if (ROOT/'telemetry').exists():shutil.copytree(ROOT/'telemetry',dest/'evidence/telemetry')
    for name in ['ready_cases.csv','blocked_paper.csv','reference_failed.csv']:
        copy(PREP/name,dest/'coverage'/name)
    copy(PREP/'dataset/dataset_manifest.json',dest/'coverage/dataset_manifest.json')
    copy(PREP/'dataset/release_lock.json',dest/'coverage/release_lock.json')
    original_rows=json.loads((WORKSPACE/'03_任务产出/P7_P11_Final_20261017/release/performance/data/manifest.json').read_text('utf8'))
    baseline_keys={r['key'] for r in original_rows}
    batch_pass=bool(summary['batch'] and summary['batch']['numerically_accepted'])
    statuses=[
        ('B01','启动、DDR 保留区与散热','CONDITIONAL','DDR no-map/系统内存排除、频率及温度已检查；USB 启动过流与 FAT 未正常卸载提示原因仍未闭环'),
        ('B02','发布包与 ARM DCT 自检','PASS','板端完整校验通过；3220项 passed/bit_exact/uses_neon 全部为 true'),
        ('B03','ARM CPU 完整回归','PARTIAL','CPU D11_rgb 1/15 双重验收通过；没有宣称 CPU 全15例通过'),
        ('B04','ARM+PL 完整回归','PASS_OVERLAP' if batch_pass else 'PARTIAL','批量包含原15例，重叠模式15/15数值与状态验收通过；同步模式仅D11_rgb' if batch_pass else '同步/重叠 D11_rgb 已通过；97例批处理尚未全量验收'),
        ('B05','同板新旧性能对照','PARTIAL','单例 CPU/sync/overlap 已测；未完成三案例交错三次及legacy对照，不能宣称最高性能'),
        ('B06','DDR 与 ARM RSS','PARTIAL','保留真实传输、进程峰值RSS和DDR地址；没有用JTAG传输速度代替算法DDR带宽'),
        ('B07','连续运行与温度','PARTIAL','批处理温度和错误轨迹已记录；不等价于独立长时压力/重复一致性验收'),
        ('B08','回退实测','PENDING','100MHz完整包部署/自检通过；尚未运行legacy或故障恢复回退实测')]
    with (dest/'board_acceptance.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.writer(f);w.writerow(['id','task','status','result']);w.writerows(statuses)
    summary['board_acceptance']=[dict(zip(['id','task','status','result'],r)) for r in statuses]
    single=summary['single_cases'];cpu=next(r for r in single if r['name']=='cpu_d11_01');sync=next(r for r in single if r['name']=='pl_sync_d11_01');overlap=next(r for r in single if r['name']=='pl_overlap_d11_01')
    summary['single_case_speedup']=dict(cpu_over_overlap=cpu['seconds_solver']/overlap['seconds_solver'],cpu_over_sync=cpu['seconds_solver']/sync['seconds_solver'],sync_seconds_saved_fraction=1-overlap['seconds_solver']/sync['seconds_solver'],scope='D11_rgb single run each, CPU 524999 kHz; solver-only')
    case_rows=[]
    if (dest/'results/dataset97_pl_01/target_suite.json').exists():
        suite=json.loads((dest/'results/dataset97_pl_01/target_suite.json').read_text('utf8'))
        source={x['key']:x for x in json.loads((PREP/'dataset/dataset_manifest.json').read_text('utf8'))['cases']}
        accepted={x['key']:x['passed'] for x in audit['cases']} if (ROOT/'dataset97_acceptance.json').exists() else {}
        for row in suite['cases']:
            key=row['key'];result=json.loads((dest/'results/dataset97_pl_01'/(key+'.json')).read_text('utf8'));data=source[key]
            case_rows.append(dict(key=key,source=data.get('source'),kind=data.get('input_kind'),paper_figures='; '.join(data.get('paper_figures',[])),baseline=key in baseline_keys,iterations=result['iterations'],seconds_solver=result['seconds_solver'],seconds_case=row['seconds'],exit_code=row['exit_code'],numerically_accepted=accepted.get(key,False),input_sha256=row['input_sha256'],model_sha256=row['model_sha256']))
        with (dest/'case_results.csv').open('w',encoding='utf-8-sig',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(case_rows[0]));w.writeheader();w.writerows(case_rows)
        summary['batch']['solver_seconds_total']=sum(x['seconds_solver'] for x in case_rows)
        summary['batch']['case_seconds_total']=sum(x['seconds_case'] for x in case_rows)
        summary['batch']['original15_accepted']=sum(x['baseline'] and x['numerically_accepted'] for x in case_rows)
    telemetry=[]
    if (ROOT/'batch_telemetry.jsonl').exists():
        telemetry=[json.loads(x) for name in ('batch_telemetry.jsonl','resume_batch_telemetry.jsonl') if (ROOT/name).exists() for x in (ROOT/name).read_text('utf8').splitlines()]
        summary['batch']['observed_max_temperature_c']=max(max(x['temperatures_now'].values()) for x in telemetry)
        summary['batch']['telemetry_samples']=len(telemetry)
    (dest/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf8')
    batch_text=(f"97/97 例已在开发板运行并通过独立数值、PFS 原始状态双重验收，包含原15例以及全部85例已适配论文测量。" if batch_pass else f"97例批处理当前状态：{summary['batch']['state']}，已运行 {summary['batch']['completed']}/97；不能据此宣称全量数值验收通过。")
    report=f'''# 最新算法实板执行报告

更新：{summary['updated']}。CPU、FPGA同步与重叠模式的 D11_rgb 均已在真实开发板跑通并通过双重验收。{batch_text}

| 执行环节 | 当前结果 |
|---|---|
| 上电与环境 | AArch64 Linux 5.4，Python 3.7.6，DDR 0x40000000起512MiB no-map，算法使用前32MiB |
| 传输与部署 | 串口控制+JTAG传输，无需以太网；完整包630571779字节，整包SHA256一致 |
| 包校验 | 性能包348文件、回退包407文件、数据包299清单文件，1057个实际归档文件全部保留 |
| ARM自检 | NEON3220项通过，bit_exact=true |
| CPU单例 | D11_rgb 2900次收敛，数值+原始状态验收通过 |
| FPGA单例 | 150MHz/burst128；同步、重叠各1例通过，无超时 |
| 扩展批量 | {batch_text} |
| 回传与图片 | 原始PFO/PFS/JSON/PPM保留；验收通过后生成FP64 NPZ、线性PNG、sRGB PNG、10倍最近邻预览 |

原会话第5–12步的逐字编号未存于工作区，本报告按已授权执行环节记录，不虚构步骤原文。此前1–4步冻结结果作为输入，历史报告保持不变。

## 单例实际测量

| D11_rgb模式 | 纯求解时间/s | 迭代数 | 相对重建误差 |
|---|---:|---:|---:|
| CPU | {cpu['seconds_solver']:.6f} | {cpu['iterations']} | {cpu['relative_normalized']:.3e} |
| FPGA同步 | {sync['seconds_solver']:.6f} | {sync['iterations']} | {sync['relative_normalized']:.3e} |
| FPGA重叠 | {overlap['seconds_solver']:.6f} | {overlap['iterations']} | {overlap['relative_normalized']:.3e} |

三次均为本次同板实跑，CPU维持524999 kHz；纯求解不含包预检、ARM自检、模型加载和FPGA配置。重叠单例相对CPU约{cpu['seconds_solver']/overlap['seconds_solver']:.2f}倍速度，相对同步少{100*(1-overlap['seconds_solver']/sync['seconds_solver']):.1f}%求解时间。每种仅一次，不能替代B05要求的三案例交错三次，也不是开发板最高性能证明。

## 完整性与数值

软件20261017-p789-p1011-v1、硬件20261016-rp2-p56-v1、ABI 0x32525001。完整1044维FP64、126×126×3输入、29×36×3输出；lambda=0.001、原容差1e-7、上限20000次均不变。性能/回退冻结源码、ELF、位流、参考和清单未修改。

验收同时检查独立参考误差、原始停止条件、PFS中的原始变量、最后一次迭代更新及线性子问题。板端来源由两次明确记录的boot ID、命令/进程记录、输入模型哈希、JTAG回传SHA256共同建立，数值相同本身不被当作板端来源证明。

## 图片与数据覆盖

批量目标为97个有匹配模型与合格参考的输入：85论文测量+12工程输入。47份论文测量仍缺匹配几何/模型/标定；1例two_axis_ramp工程压力输入的独立参考在20000次未达原容差，不进入合格批量。具体文件见coverage/，不得说成所有132份论文测量都已重建。

论文版面截图、GT、已有重建图不是墙面传感器测量，不送入反演器。35帧单帧序列各自使用已记录的累计尺度/35，这是推导尺度，非独立物理亮度标定。输入诊断的uncalibrated标记仍保留。

图片目录为[images/dataset97_pl_01/index.html](images/dataset97_pl_01/index.html)（仅批量验收完成后存在）。图像保持原行列顺序；显示使用三通道共用最大值，额外提供sRGB传递函数版本，所有映射有display_metadata.json。10倍预览仅最近邻放大，不增加原始分辨率。原始PFO/PFS及FP64数组用于科学比较。

## 连接、运行状态与限制

串口COM10/115200与JTAG已实际完成双向传输；以太网仍NO-CARRIER不阻塞本批任务。0x50000000传输缓冲在原no-map保留区内，与算法内存不重叠；使用排他锁，JTAG传输未暂停ARM核。上午完成52例后按用户要求安全关机，下午重新上电，校验包与断点后仅续跑45例。两段分别回传验收，合并时逐项核对文件SHA256及输入/模型身份，来源见results/dataset97_pl_01/merge_provenance.json。JTAG30MHz仅用于传输，最终恢复状态见日志。

传输最后一块非对齐内存复制曾触发Python SIGBUS，已用独立子进程复现、修复为64字节对齐后核验原字节哈希；回传的msync不受该映射支持，改为对齐写入后回读验证。故障只发生在传输工具，未改算法或吞掉数值错误。证据见evidence/local/transport_alignment_incident.json。

恢复shell的/var/lock曾指向不存在的/var/volatile/lock，本次补建运行目录以使用原发布锁。原5个启动配置已备份，不更改开机自启；最终逐文件比对结果见evidence/。上午开机USB过流2条提示及FAT未正常卸载提示原因尚未闭环，不能记为完全排除；没有执行fsck。午间已卸载启动分区、根分区转只读并收到内核Power down，之后由用户重新上电。相机实时采集、自定位自动选模、当前算法到实时UI仍未集成。

## 板端验收边界

详见[board_acceptance.csv](board_acceptance.csv)。B02已通过；B03仅CPU1/15；B04以实际批量验收为准；B05/B06/B07仅已有测量子项，B08未执行。不能把本轮目标完成写成B01–B08全部通过。更紧1e-8参考下new_78901的既有差异限制仍保留。

板端工作目录：/home/root/ps_runs_20261017/steps5_12_01。全部原始结果留在板端。正式交付应同时位于03_任务产出/Board_Steps_5_12_20261017和02_代码仓库/penumbrasight/sim/results/board_steps_5_12_20261017；工具副本进board/full_model/session_tools/20261017。本报告与当前状态入口更新，冻结历史发布报告不回写实板状态。
'''
    if (ROOT/'display/display_state.json').exists():
        summary['display']=json.loads((ROOT/'display/display_state.json').read_text('utf8'))
        if (ROOT/'display/final_display_state.json').exists():summary['display']=json.loads((ROOT/'display/final_display_state.json').read_text('utf8'))
        report+='\n## 显示屏实显与逐例自动更新\n\n用户追加要求已执行：DP连接、RGB565。上午分辨率1024×768；下午重启后为1280×800，由live_display_resume_v2.py在原生帧缓冲内居中放置1024×768结果卡。每当target_suite记录一个新完成的案例，就从原PFO读取重建图、验证CRC/身份/原停止残差并更新屏幕。新结果显示AUDIT:PENDING；独立验收通过后以PFO哈希清单标记PASSED。下午先显示已验收第52例，再依次显示第53–97例，最终更新为97例全部通过。查看display/实际帧缓冲截图及最终板端display/live_history.jsonl，可核对逐例更新记录。\n\n屏幕标注原始分辨率、模式、求解时间、进度及验收状态。上午自动更新从第24例开始，不宣称前23例已逐张展示。tty1由文本模式切到图形模式以消除控制台光标叠加，原模式和原屏幕内容均已备份；串口控制台不受影响，重启仍用原配置。帧缓冲回读SHA256核对通过。这是文件批量结果的自动展示，不等价于相机实时采集/整机UI系统集成。显示与串口监测产生额外负载，批量计时不作为纯性能基准。\n'
        (dest/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf8')
    (dest/'执行报告.md').write_text(report,encoding='utf8')
    (dest/'tools/README.md').write_text('本目录记录2026-10-17板端会话工具及实际命令。固定COM10、PSU、0x50000000和会话路径仅适用于本次已核实的板卡内存保留配置；先读执行报告及脚本，不盲目重复启动或覆盖结果。所有长任务后台运行并保存状态；传输与结果回传需排他使用串口、JTAG和算法锁。发布包及数据集保持原哈希。\n',encoding='utf8')
    print(json.dumps(dict(delivery=str(dest),summary=summary),ensure_ascii=False,indent=2),flush=True)

if __name__=='__main__':main()
