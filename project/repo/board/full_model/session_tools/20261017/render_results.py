"""Render accepted board reconstructions without changing scientific FP64 output."""
from pathlib import Path
import argparse,hashlib,json,struct,zlib,html
import numpy as np
from PIL import Image,ImageDraw

def main():
    p=argparse.ArgumentParser();p.add_argument('--results',type=Path,required=True);p.add_argument('--acceptance',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    report=json.loads(a.acceptance.read_text('utf8'))
    if report['passed'] is not True:raise ValueError('Acceptance failed')
    a.output.mkdir(parents=True,exist_ok=False);rows=[];tiles=[]
    dataset=Path(r'D:\学习\26秋嵌入式\PenumbraSight工作区\03_任务产出\Board_Steps_1_4_20261017\dataset\dataset_manifest.json')
    sources={r['key']:r for r in json.loads(dataset.read_text('utf8'))['cases']}
    for case in report['cases']:
        if case['passed'] is not True:raise ValueError('Case not accepted')
        key=case['key'];source=a.results/(key+'.pfo');data=source.read_bytes()
        if len(data)!=75232 or data[:8]!=b'PSFULL1\0' or struct.unpack_from('<IIII',data,8)!=(1,4,29,36) or zlib.crc32(data[64:])!=struct.unpack_from('<I',data,32)[0]:raise ValueError('Invalid PFO')
        arrays=np.frombuffer(data[64:],dtype='<f8').reshape(3,29,36,3)
        if not np.isfinite(arrays).all():raise ValueError('Nonfinite PFO')
        out=a.output/key;out.mkdir();np.savez(out/'reconstruction_fp64.npz',h=arrays[0],normalized=arrays[1],physical=arrays[2])
        maximum=float(np.max(arrays[2]));scaled=np.clip(arrays[2]/maximum if maximum>0 else arrays[2],0,1)
        linear=np.rint(scaled*255).astype(np.uint8)
        srgb=np.where(scaled<=.0031308,12.92*scaled,1.055*np.power(scaled,1/2.4)-.055)
        image=Image.fromarray(np.rint(np.clip(srgb,0,1)*255).astype(np.uint8),'RGB')
        Image.fromarray(linear,'RGB').save(out/'reconstruction_linear.png');image.save(out/'reconstruction_srgb.png')
        image.resize((360,290),Image.Resampling.NEAREST).save(out/'preview_10x.png')
        row=dict(key=key,source_pfo_sha256=hashlib.sha256(data).hexdigest(),shape=[29,36,3],orientation='native PFO row/column order; no transpose or flip',display_source='physical FP64 RGB',display_divisor=maximum,display_scale='one shared positive maximum across all three channels; no per-channel normalization',linear_png='round(255*clip(physical/divisor,0,1))',srgb_png='standard sRGB transfer after shared scaling; visualization only',preview='nearest-neighbor 10x; no new resolution or detail')
        source_info=sources.get(key,{})
        row['input_source']=source_info.get('source');row['paper_figures']=source_info.get('paper_figures',[]);row['input_kind']=source_info.get('input_kind')
        (out/'display_metadata.json').write_text(json.dumps(row,indent=2),encoding='utf8');rows.append(row)
        tile=Image.new('RGB',(252,233),'#202020');tile.paste(image.resize((252,203),Image.Resampling.NEAREST),(0,0));ImageDraw.Draw(tile).text((4,210),key[:40],fill='white');tiles.append(tile)
    columns=5;sheet=Image.new('RGB',(columns*252,((len(tiles)+columns-1)//columns)*233),'#202020')
    for i,tile in enumerate(tiles):sheet.paste(tile,((i%columns)*252,(i//columns)*233))
    sheet.save(a.output/'contact_sheet.jpg',quality=93)
    (a.output/'manifest.json').write_text(json.dumps(dict(cases=rows,count=len(rows),physical_board_origin='See verified raw results and session evidence'),ensure_ascii=False,indent=2),encoding='utf8')
    cards=[]
    for row in rows:
        key=html.escape(row['key']);source=html.escape(str(row.get('input_source') or ''));figures=html.escape(', '.join(row['paper_figures']))
        cards.append('<article><a href="'+key+'/preview_10x.png"><img loading="lazy" src="'+key+'/reconstruction_srgb.png" alt="'+key+'"></a><h2>'+key+'</h2><p>'+source+'</p><p>'+figures+'</p><nav><a href="'+key+'/reconstruction_linear.png">线性显示</a> · <a href="'+key+'/reconstruction_srgb.png">sRGB 显示</a> · <a href="'+key+'/reconstruction_fp64.npz">原始 FP64</a> · <a href="'+key+'/display_metadata.json">显示参数</a></nav></article>')
    page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>板端重建结果</title><style>body{font:16px system-ui,sans-serif;margin:32px;background:#f3f5f8;color:#16243a}header{max-width:1000px;margin-bottom:28px}h1{font-size:28px}input{padding:12px;width:min(90%,600px);font:inherit;border:1px solid #adb7c4;border-radius:8px}main{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:18px}article{background:white;padding:16px;border-radius:12px;overflow-wrap:anywhere}img{width:100%;max-width:360px;image-rendering:pixelated}h2{font-size:16px}p,nav{font-size:13px;line-height:1.6}a{color:#1258a3}footer{margin-top:30px}</style><header><h1>开发板重建结果 · '''+str(len(rows))+''' 例</h1><p>这些结果来自开发板上的完整 1044 维 FP64 算法，并通过独立数值与求解状态验收。原始分辨率为 36 × 29 像素；放大图采用最近邻显示。sRGB 图为便于查看的亮度映射，科学数值保存在 FP64 文件中。</p><input id="search" type="search" placeholder="按案例、源文件或论文图号筛选" aria-label="筛选重建结果"><p id="count"></p></header><main>'''+''.join(cards)+'''</main><footer><p>覆盖范围：当前已有匹配模型与合格参考的数据。未处理的数据缺口和运行条件见同批执行报告。</p></footer><script>const q=document.querySelector('#search'),cards=[...document.querySelectorAll('article')];function filter(){let n=0;for(const c of cards){c.hidden=!c.textContent.toLowerCase().includes(q.value.toLowerCase());if(!c.hidden)n++;}document.querySelector('#count').textContent='显示 '+n+' / '+cards.length+' 例';}q.addEventListener('input',filter);filter();</script></html>'''
    (a.output/'index.html').write_text(page,encoding='utf8')
    print('RENDERED',len(rows),str(a.output),flush=True)

if __name__=='__main__':main()
