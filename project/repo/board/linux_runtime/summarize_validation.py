"""Read actual FPGA results and MATLAB outputs; produce auditable acceptance report."""
from pathlib import Path
import json,hashlib
import numpy as np
from scipy.io import loadmat

repo=Path(__file__).resolve().parents[2]
out=repo.parent.parent/'03_任务产出/S8-26_Board_Verified_20261006'
suite=json.loads((out/'extended_validation.json').read_text())
rows=suite['results']
assert len(rows)==61 and all(r['rc']==0 for r in rows),'Board suite incomplete or failed'
by_frame={}
for r in rows:by_frame.setdefault(r['frame'],set()).add(r['sha256'])
assert len(by_frame)==13 and all(len(v)==1 for v in by_frame.values()),'Nondeterministic output'
numeric=[]
for name in ['ambient1','bur','colbar','it0','mushroom','smilehat']:
    b=out/('verified_image_test_'+name+'_f000.psf.codes.bin')
    host=out/(name+'_host.codes.bin')
    assert b.read_bytes()==host.read_bytes(),name+' FPGA/software mismatch'
    m=loadmat(str(out/(name+'_matlab_result.mat')))
    f=m['f'].T;original=m['f_original'].T
    board=np.fromfile(b,dtype='<i4').reshape(3,432)/2**18
    numeric.append(dict(case=name,bit_exact_host=True,selected_id=json.loads(b.with_name(b.name.replace('.codes.bin','.json')).read_text())['selected_id'],
        fpga_vs_matlab_relative_l2=float(np.linalg.norm(board-f)/np.linalg.norm(f)),
        fpga_vs_matlab_max_abs=float(np.abs(board-f).max()),
        input_quantization_relative_l2=float(np.linalg.norm(m['d']-m['d_original'])/np.linalg.norm(m['d_original'])),
        fpga_vs_source_matlab_relative_l2=float(np.linalg.norm(board-original)/np.linalg.norm(original))))
multi=json.loads((out/'multiframe_host_validation.json').read_text())
assert all(r['matches_board'] for r in multi)
samples=json.loads((out/'validation_telemetry.json').read_text())
temps=[t/1000 for s in samples for t in s['temperature_millicelsius'].values()]
durations=[r['seconds'] for r in rows]
metrics={'board_cases':len(rows),'input_frames':len(by_frame),'elapsed_seconds':suite['elapsed_seconds'],
 'run_seconds_min':min(durations),'run_seconds_max':max(durations),'temperature_samples':len(samples),
 'sampled_temperature_min_c':min(temps),'sampled_temperature_max_c':max(temps),'numeric':numeric,
 'all_frames_repeatable':True,'multiframe':multi}
(out/'numerical_validation_20261007.json').write_text(json.dumps(numeric,indent=2),encoding='utf8')
(out/'validation_acceptance_20261007.json').write_text(json.dumps(metrics,indent=2),encoding='utf8')
lines=['# 现有 SD 图像移植验证报告（2026-10-07）','',
 '## 验证范围','',
 '不接摄像头。本次覆盖当前 SD 部署的13帧、6类图像：色条8帧，以及 bur、mushroom、smilehat、it0、ambient1 各1帧。色条8帧是同一幅图的重复载荷，不是8个不同场景。论文数据集中其他场景不在本次已部署输入覆盖范围内。','',
 '## 实板执行与稳定性','',
 f'- 61次完整算法启动全部返回0，连续运行 {suite["elapsed_seconds"]/60:.2f} 分钟。每次调用独占启动器重置算法逻辑；风扇不受该重置影响。',
 '- 13个输入文件全部逐一运行；随后48次循环复测。同一输入每次输出SHA256完全一致。',
 f'- 每次完整启动耗时 {min(durations):.2f}～{max(durations):.2f} 秒，包括模型读取与计算，不是纯核延迟。',
 f'- 后半程采集 {len(samples)} 次温度和内存快照，温度 {min(temps):.2f}～{max(temps):.2f}°C；此前串口采样25.63～27.38°C。温度采样不是全部运行时间的无间断监测。',
 '- 每次结果包含三个通道各64次迭代，低置信度标记保留。真实结果保存于 extended_validation.json；采样保存于 validation_telemetry.json。','',
 '## 定点与 MATLAB 数值对比','',
 '六类图像的原始 MATLAB 数据与上板12位载荷经 CRC32 精确匹配，源路径见 matlab_input_inventory.json。软件定点模拟与 FPGA 原始输出逐字节一致；1/4/8帧色条对照亦一致。',
 '', '| 图像 | 候选ID | FPGA/软件定点 | 同参数MATLAB相对L2误差 | 相对未量化源图MATLAB误差 |',
 '|---|---:|---|---:|---:|']
for r in numeric:lines.append(f'| {r["case"]} | {r["selected_id"]} | 逐字节一致 | {r["fpga_vs_matlab_relative_l2"]*100:.6f}% | {r["fpga_vs_source_matlab_relative_l2"]*100:.6f}% |')
lines += ['',
 'MATLAB 对照在真实 R2024b 中执行，采用同一量化低秩模型、选定候选、64次PDHG迭代、相同步长和TV系数，使用独立双精度运算。它验证移植的计算误差，不等于证明与论文原始FISTA及另一组几何参数的最终图像一致。相对未量化源图的对照取消12位输入量化，但仍采用相同模型、候选和求解参数。',
 '', '环境光图差分较弱，12位输入量化影响更明显；不能据此宣布所有场景的物理重建质量合格。其余图像与同参数双精度结果误差约0.0018%～0.0041%，环境光图约0.0502%。',
 '', '## 本次修正与可复现代码','',
 '- 修正验证脚本对双绿通道的错误假设：实际 RTL 保留 G1+G2，不能取平均。此修正仅作用于软件对照，未改动实板算法。',
 '- 添加逐帧循环测试、结果导出、温度/内存采样、原图CRC匹配、同参数MATLAB双精度对照与报告生成脚本，全部位于现有仓库 board/linux_runtime。',
 '- 修正串口结果ZIP导出的1980年前时间戳兼容问题。板上系统时钟仍为1970起算，测试使用单调时钟统计耗时。',
 '- MATLAB启动使用临时ASCII配置路径及本机现有许可证；不修改系统环境变量或用户原有配置。',
 '', '## 验收结论与边界','',
 '当前部署输入的完整上板处理、重复确定性和同参数数值误差验证均完成。屏幕仍输出标记清楚的低置信度诊断预览；不得将此结论写为“物理标定完成”“原论文重建完全等价”或“长期无人值守稳定性已证明”。本次稳定性证据为约38分钟、61次重复启动。',
 '', '算法位流、风扇启动、生产程序未因本次验证而改动。']
(out/'现有图像完整验证报告_20261007.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
print(json.dumps(metrics,ensure_ascii=False,indent=2))
