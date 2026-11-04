"""Prepare a small font atlas and title patches for the board-native live viewer."""
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
import numpy as np,base64,json,zlib
ROOT=Path(__file__).resolve().parent
out=ROOT/'display';out.mkdir(exist_ok=True)
font=ImageFont.truetype('C:/Windows/Fonts/consola.ttf',22)
atlas={}
for n in range(32,127):
    im=Image.new('L',(14,28));ImageDraw.Draw(im).text((0,-1),chr(n),font=font,fill=255)
    atlas[chr(n)]=[y*14+x for y in range(28) for x in range(14) if im.getpixel((x,y))>=96]
patches=[]
for x,y,w,h,text,size,color,background in [(26,62,980,32,'最新生成结果自动显示 · 独立数值验收待完成',17,'#f2c87d','#1c2c43'),(26,694,980,36,'每完成一例自动更新；批量与验收仍在后台进行。',21,'#f2f6fb','#101a2b'),(26,736,980,27,'图像来自开发板 PFO 原始结果；原始 36 × 29，sRGB 最近邻显示。',15,'#9daec5','#101a2b')]:
    im=Image.new('RGB',(w,h),background);ImageDraw.Draw(im).text((3,0),text,font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',size),fill=color)
    p=np.asarray(im,dtype=np.uint16);packed=((p[:,:,0]>>3)<<11)|((p[:,:,1]>>2)<<5)|(p[:,:,2]>>3)
    patches.append(dict(x=x,y=y,width=w,height=h,data=base64.b64encode(zlib.compress(packed.astype('<u2').tobytes(),9)).decode()))
data=json.dumps(dict(atlas=atlas,patches=patches),separators=(',',':')).encode()
(out/'live_assets.json.z').write_bytes(zlib.compress(data,9))
print('LIVE_ASSETS_BYTES',len(zlib.compress(data,9)))
