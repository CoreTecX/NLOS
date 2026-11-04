"""Portable historical-result gallery, refreshed after each committed result."""
from pathlib import Path
import html
from batch_store import atomic_bytes,atomic_json,verify_seal

def publish_gallery(root,checkpoint):
 root=Path(root);rows=checkpoint['items'];complete=[r for r in rows if r['status']=='COMPLETE'];latest=complete[-1] if complete else None
 if latest:verify_seal(root,latest)
 table=[]
 for row in rows:
  key=html.escape(row['id']);status=html.escape(row['status']);reason=html.escape(str(row.get('error_code') or ''))
  link=html.escape(row['output']+'/index.html',quote=True) if row.get('output') else ''
  table.append('<tr><td>'+key+'</td><td>'+status+'</td><td>'+reason+'</td><td>'+('<a href="'+link+'">结果与验收</a>' if link else '')+'</td></tr>')
 image='等待第一项完整重建通过验收。'
 if latest:
  image='<h2>最近完成：'+html.escape(latest['id'])+'</h2><p>历史数据 · 完整算法重建 · 序号 '+str(latest['index']+1)+' · 非实时相机预览</p><img src="'+html.escape(latest['output']+'/reconstruction_native.png',quote=True)+'">'
 body='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta http-equiv="refresh" content="3"><title>完整重建结果</title><style>body{background:#14212b;color:#edf4fa;font:17px sans-serif;margin:30px}img{width:576px;max-width:100%;image-rendering:pixelated}td,th{padding:8px;text-align:left;border-bottom:1px solid #486071}a{color:#82d7ed}</style><h1>PenumbraSight · 批量完整重建</h1><p>每个通过验收的新结果保存后更新。显示图用于观察，原始数值及来源见各项记录。失败或阻塞项不会显示为成功。</p>'+image+'<table><tr><th>输入</th><th>状态</th><th>原因</th><th>记录</th></tr>'+''.join(table)+'</table></html>'
 atomic_bytes(root/'index.html',body.encode('utf8'))
 atomic_json(root/'latest_result.json',dict(kind='HISTORICAL_COMPLETE_RECONSTRUCTION',latest=latest,complete_count=len(complete),physical_display_measured=False))
 return dict(status='PUBLISHED',complete_count=len(complete),latest_id=latest['id'] if latest else None)
