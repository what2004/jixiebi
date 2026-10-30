#!/usr/bin/env python3
"""离线检查握笔预设并从实际 STL 网格渲染四视图；依赖 numpy 和 Pillow。

运行 python3 inspect_pen_grasp.py --render。
几何检查使用全部网格顶点、三角形中心和边中点，不等同于动力学或完整碰撞检测。
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import math,json,struct
ROOT=Path(__file__).resolve().parents[1]
URDF=ROOT/'chapttan6_ws/src/shoubi_description/urdf/hand_l20.urdf.xacro'
MESH=URDF.parent.parent/'meshes'
robot=ET.parse(URDF).getroot()
joints=robot.findall('joint')
def rot(a,t):
 a=np.array(a,dtype=float);a/=np.linalg.norm(a)
 x,y,z=a;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
 return np.eye(3)+np.sin(t)*K+(1-np.cos(t))*(K@K)
def origin(j):
 o=j.find('origin');T=np.eye(4)
 r,p,y=map(float,o.get('rpy','0 0 0').split());T[:3,:3]=rot([0,0,1],y)@rot([0,1,0],p)@rot([1,0,0],r)
 T[:3,3]=list(map(float,o.get('xyz').split()));return T
JOINTS=[(j.get('name'),j.find('parent').get('link'),j.find('child').get('link'),origin(j),list(map(float,j.find('axis').get('xyz').split()))) for j in joints]
def fk(q):
 T={'hand_base_link':np.eye(4)}
 for name,p,c,o,a in JOINTS:
  R=np.eye(4);R[:3,:3]=rot(a,q.get(name,0.));T[c]=T[p]@o@R
 return T

import argparse
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--render',action='store_true')
args=parser.parse_args()
preset=json.loads((ROOT/'joint_space_demo/pen_grasp.json').read_text())
q=preset['goal']
from jt_core import LIMITS, JOINTS_ARM, JOINTS_HAND
assert set(q)==set(JOINTS_ARM+JOINTS_HAND)
for name,value in q.items():
    assert math.isfinite(value) and LIMITS[name][0] <= value <= LIMITS[name][1],name
c=np.array(preset['pen_xyz'])
r,p,y=preset['pen_rpy']
R=rot([0,0,1],y)@rot([0,1,0],p)@rot([1,0,0],r)
u,v,axis=R[:,0],R[:,1],R[:,2]
output_dir=ROOT/'traj/pen_grasp'
output_dir.mkdir(parents=True,exist_ok=True)

def clearance(points):
    delta=points-c
    axial=delta@axis
    radius=np.linalg.norm(delta-axial[:,None]*axis,axis=1)
    def cylinder_sdf(radial,along):
        z=np.stack([radial,along],axis=1)
        return np.linalg.norm(np.maximum(z,0),axis=1)+np.minimum(np.max(z,axis=1),0)
    barrel=cylinder_sdf(radius-.006,np.abs(axial+.02)-.08)
    # 包络 40 层台阶，比真实笔尖略大，作为保守检查。
    cone_radius=.006*np.clip((.08-axial)/.02,0,1)+.00015
    tip=cylinder_sdf(radius-cone_radius,np.abs(axial-.07)-.01)
    return np.minimum(barrel,tip)

T=fk(q)
report={'method':'dense_mesh_surface_samples_against_pen_envelope',
        'pen_length_mm':180,'pen_diameter_mm':12,'links':{},'limitations':
        '仅用于静态可视化的表面采样检查，不验证抓取力、摩擦或完整自碰撞。'}
for name,t in T.items():
    path=MESH/(name+'.STL')
    if not path.exists():continue
    data=path.read_bytes()
    faces=np.frombuffer(data,dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]),offset=84)['v'].astype(float)
    samples=np.vstack([faces.reshape(-1,3),faces.mean(axis=1),
                       (faces[:,0]+faces[:,1])/2,(faces[:,1]+faces[:,2])/2,(faces[:,2]+faces[:,0])/2])
    samples=samples@t[:3,:3].T+t[:3,3]
    gap=clearance(samples)
    nearest=int(np.argmin(gap))
    axial=float((samples[nearest]-c)@axis)
    report['links'][name]={'minimum_sample_clearance_mm':round(float(gap.min()*1000),4),
                          'inside_samples':int(np.sum(gap<0)),'samples':len(samples),
                          'nearest_point_distance_from_tip_mm':round((.08-axial)*1000,3)}
report['inside_samples_total']=sum(row['inside_samples'] for row in report['links'].values())
for name in ['thumb_distal','index_distal','middle_distal']:
    print(name,report['links'][name])
print('inside samples:',report['inside_samples_total'])
(output_dir/'geometry_check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
assert report['inside_samples_total']==0,'表面采样发现笔与手部交叠'
if not args.render:
    raise SystemExit(0)
from PIL import Image, ImageDraw, ImageFont
T=fk(q)
triangles=[];colors=[]
for link in robot.findall('link'):
 name=link.get('name')
 if name not in T:continue
 for visual in link.findall('visual'):
  mesh=visual.find('geometry/mesh')
  if mesh is None:continue
  path=MESH/Path(mesh.get('filename')).name
  data=path.read_bytes();n=struct.unpack_from('<I',data,80)[0]
  f=np.frombuffer(data,dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]),offset=84,count=n)['v'].astype(float)
  f=f@T[name][:3,:3].T+T[name][:3,3]
  triangles.append(f)
  base=[173,185,199] if 'metacarpals' not in name or 'thumb' in name else [70,78,91]
  colors.append(np.tile(base,(len(f),1)))
def cylinder(z0,z1,r0,r1,color):
 f=[]
 for i in range(64):
  a=2*np.pi*i/64;b=2*np.pi*(i+1)/64
  p=c+z0*axis+r0*(np.cos(a)*u+np.sin(a)*v)
  q1=c+z0*axis+r0*(np.cos(b)*u+np.sin(b)*v)
  r=c+z1*axis+r1*(np.cos(a)*u+np.sin(a)*v)
  s=c+z1*axis+r1*(np.cos(b)*u+np.sin(b)*v)
  f.extend([[p,q1,s],[p,s,r],[c+z0*axis,q1,p],[c+z1*axis,r,s]])
 triangles.append(np.array(f));colors.append(np.tile(color,(len(f),1)))
cylinder(-.1,.06,.006,.006,[30,108,222])
for i in range(40):cylinder(.06+i*.0005,.06+(i+1)*.0005,.006*(40-i)/40,.006*(40-i)/40,[48,51,58])
f=np.vstack(triangles);colors=np.vstack(colors)
normals=np.cross(f[:,1]-f[:,0],f[:,2]-f[:,0]);normals/=np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-12)
W=850
out=Image.new('RGB',(2*W,2*W),(242,245,250))
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',22)
for idx,(eye,label) in enumerate([([.2,1,.2],'Thumb and index'),([1,-.9,.35],'Middle finger support'),([1,.8,.35],'Palm view'),([1,0,1.3],'Finger alignment')]):
 d=np.array(eye,dtype=float);d/=np.linalg.norm(d)
 right=np.cross([-1.,0,0],d);right/=np.linalg.norm(right);up=np.cross(d,right)
 center=np.array([.035,.005,.14]);scale=2400.
 local=f-center
 xy=np.stack([local@right,local@up],axis=-1)*scale
 xy[:,:,0]+=W/2;xy[:,:,1]=W/2-xy[:,:,1]
 depth=(local@d).mean(axis=1)
 shade=.65+.35*np.abs(normals@d)
 rgb=np.clip(colors*shade[:,None],0,255).astype(int)
 area=np.abs(np.cross(xy[:,1]-xy[:,0],xy[:,2]-xy[:,0]))
 visible=(area>.08)&(xy[:,:,0].max(1)>=0)&(xy[:,:,0].min(1)<W)&(xy[:,:,1].max(1)>=0)&(xy[:,:,1].min(1)<W)
 im=Image.new('RGB',(W,W),(242,245,250));draw=ImageDraw.Draw(im)
 for k in np.argsort(depth):
  if visible[k]:draw.polygon([tuple(p) for p in xy[k]],fill=tuple(rgb[k]))
 draw.text((20,20),label,fill=(35,50,65),font=font)
 draw.text((20,W-40),'180 mm pen / 12 mm barrel',fill=(35,50,65),font=font)
 out.paste(im,((idx%2)*W,(idx//2)*W))
path=output_dir/'pen_grasp_views.png';out.save(path);print(path)
