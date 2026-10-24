"""Pure numerical metrics; truth is used only in post-hoc evaluation."""
import numpy as np

def local_displacement_error(pg0,pg1,Rg0,pe0,pe1,Re0):
    return Rg0@Re0.T@(pe1-pe0)-(pg1-pg0)

def translation_axis(H_lidar,R_body_lidar,R_world_body,forward):
    H=R_body_lidar@H_lidar[3:,3:]@R_body_lidar.T
    H=(H+H.T)*.5
    eig,U=np.linalg.eigh(H)
    if eig[-1]<=0 or eig[0]<-1e-6*eig[-1]:raise ValueError('Invalid translational information')
    u=R_world_body@U[:,0]
    if abs(u@forward)>1e-8:
        if u@forward<0:u=-u
    elif u[np.argmax(abs(u))]<0:u=-u
    return u,eig,np.clip(eig[0]/eig[-1],0,1)

def stats(values):
    a=np.asarray(values,float)
    if not len(a):return dict(n=0,bias=None,std=None,rmse=None)
    return dict(n=len(a),bias=float(a.mean()),std=float(a.std()),rmse=float(np.sqrt(np.mean(a*a))))
