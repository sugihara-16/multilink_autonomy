"""ROS1 serialization for existing FAST inputs, usable from the ROS2 reader."""
import struct
import numpy as np
CAMERAS=('link1','link3','link4')
DTYPE=np.dtype({'names':['x','y','z','intensity','t','line','ring','reflectivity','ambient','tag','range'],'formats':['<f4']*4+['<u4','u1','u1','<u2','<u2','u1','<u4'],'offsets':[0,4,8,12,16,20,21,22,24,26,28],'itemsize':32})
def string(s):
 b=s.encode();return struct.pack('<I',len(b))+b
def header(t,frame):return struct.pack('<III',0,*divmod(int(t),10**9))+string(frame)
def packet(i,b):return struct.pack('<II',i,len(b))+b
def imu(t,v):
 return header(t,'slam_imu')+struct.pack('<37d',*[0.]*4,-1.,*[0.]*8,*v[3:],*[0.]*9,*v[:3],*[0.]*9)
def image(t,j,gray):
 data=np.repeat(gray[:,:,None],3,axis=2).tobytes()
 return header(t,f'cam_{CAMERAS[j]}_optical')+struct.pack('<II',360,480)+string('bgr8')+struct.pack('<BII',0,1440,len(data))+data
def cloud(t,v):
 r=np.linalg.norm(v[:,:3],axis=1);v=v[np.isfinite(v[:,:3]).all(axis=1)&(r>=.8)&(r<=30)]
 a=np.zeros(len(v),DTYPE)
 for j,k in enumerate(('x','y','z','intensity')):a[k]=v[:,j]
 a['line']=v[:,4];a['ring']=v[:,4];a['range']=(np.linalg.norm(v[:,:3],axis=1)*1000).astype(np.uint32)
 fields=b''
 for n in ('x','y','z','intensity','t','reflectivity','ring'):
  fields+=string(n)+struct.pack('<IBI',DTYPE.fields[n][1],7 if n in ('x','y','z','intensity') else 6 if n=='t' else 4 if n=='reflectivity' else 2,1)
 return header(t,'slam_lidar')+struct.pack('<III',1,len(v),7)+fields+struct.pack('<BIII',0,32,len(v)*32,a.nbytes)+a.tobytes()+b'\x01'
