import trimesh,numpy as np,time,json
from scipy import ndimage
from pathlib import Path
m=trimesh.load('/tmp/zab_collision.ply',process=False)
lo=np.floor(m.bounds[0])-[2,2,2];hi=np.ceil(m.bounds[1])+[2,2,2];h=.5
shape=tuple((np.ceil((hi-lo)/h).astype(int)+1).tolist())
print('shape',shape,flush=True)
v=m.voxelized(h,max_iter=None).points
print('voxels',len(v),flush=True)
occ=np.zeros(shape,bool);idx=np.rint((v-lo)/h).astype(int);occ[tuple(idx.T)]=True
# Fill one-voxel pinholes in measured surfaces, not tunnel openings.
loz=occ.argmax(2);hiz=shape[2]-1-occ[:,:,::-1].argmax(2);z=np.arange(shape[2])[None,None,:]
inside=occ.any(2)[:,:,None]&(z>loz[:,:,None])&(z<hiz[:,:,None])
d=ndimage.distance_transform_edt(~occ,sampling=h).astype('float32');d[~inside]=0
np.savez_compressed('/tmp/zab_voxels.npz',occupied=occ,distance=d,lower=lo,pitch=h)
print('saved',flush=True)
