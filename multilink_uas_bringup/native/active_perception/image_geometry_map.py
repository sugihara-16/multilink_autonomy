"""Planning-only image landmarks with LiDAR-supported depth; no estimator input."""
import cv2
import numpy as np
from scipy.spatial import cKDTree


def depth_at_pixels(pixels,cloud_body,Rbc,tbc,K,range_max=30.):
    """Intersect image rays with small, verified planar LiDAR neighborhoods.

    Rejected neighborhoods stay absent rather than acquiring invented depth.
    Covariances are conservative scoring approximations, never filter factors.
    """
    pixels=np.asarray(pixels,dtype=float).reshape(-1,2)
    points=np.full((len(pixels),3),np.nan);cov=np.full((len(pixels),3,3),np.nan)
    pc=(np.asarray(cloud_body)-tbc)@Rbc
    pc=pc[np.isfinite(pc).all(axis=1)&(pc[:,2]>.2)&(np.linalg.norm(pc,axis=1)<range_max)]
    if len(pc)<8 or not len(pixels):return points,cov
    uv=pc[:,:2]/pc[:,2,None]*np.array([K[0,0],K[1,1]])+K[:2,2]
    distances,indices=cKDTree(uv).query(pixels,k=8)
    for i,(pixel,ds,ii) in enumerate(zip(pixels,distances,indices)):
        if ds[0]>8 or ds[4]>18:continue
        neighbors=pc[ii[ds<=18]]
        center=neighbors.mean(axis=0);centered=neighbors-center
        values,vectors=np.linalg.eigh(centered.T@centered/len(neighbors));normal=vectors[:,0]
        # Do not bridge foreground/background depth discontinuities or curved
        # regions with an unsupported plane. Require two-dimensional support.
        if values[1]<1e-5 or np.sqrt(max(values[0],0))>.06:continue
        if np.linalg.norm(centered,axis=1).max()>max(.6,.15*np.linalg.norm(center)):continue
        ray=np.linalg.solve(K,np.r_[pixel,1.]);ray/=np.linalg.norm(ray)
        incidence=float(normal@ray)
        if abs(incidence)<.2:continue
        depth=float(normal@center)/incidence
        if not .3<depth<range_max:continue
        hit=depth*ray
        if np.linalg.norm(hit-center)>max(.3,.1*depth):continue
        sigma_depth=max(.08,(.03+np.sqrt(max(values[0],0)))/abs(incidence))
        bearing=(2./min(K[0,0],K[1,1]))**2
        C=sigma_depth**2*np.outer(ray,ray)+depth**2*bearing*(np.eye(3)-np.outer(ray,ray))
        points[i]=Rbc@hit+tbc;cov[i]=Rbc@C@Rbc.T
    return points,cov


class ImageGeometryMap:
    """Persistent optical-flow tracks, including corners not selected by ROVIO."""
    def __init__(self,K,max_features=180):
        cv2.setNumThreads(1)
        self.K=np.asarray(K);self.max_features=max_features
        self.previous=None;self.pixels=np.empty((0,2),np.float32)
        self.ids=np.empty(0,dtype=np.int64);self.next_id=0
        self.anchors={}

    def update(self,gray,cloud_body,Rbc,tbc,range_max,T_W_B=None):
        gray=np.ascontiguousarray(gray,dtype=np.uint8)
        h,w=gray.shape
        if self.previous is not None and len(self.pixels):
            after,ok,_=cv2.calcOpticalFlowPyrLK(self.previous,gray,self.pixels,None,winSize=(21,21),maxLevel=3)
            back,reverse,_=cv2.calcOpticalFlowPyrLK(gray,self.previous,after,None,winSize=(21,21),maxLevel=3)
            good=ok.reshape(-1).astype(bool)&reverse.reshape(-1).astype(bool)&(np.linalg.norm(back-self.pixels,axis=1)<.75)
            good&=(after[:,0]>=8)&(after[:,0]<w-8)&(after[:,1]>=8)&(after[:,1]<h-8)
            self.pixels=after[good];self.ids=self.ids[good]
        mask=np.zeros_like(gray);mask[8:-8,8:-8]=255
        for x,y in self.pixels:cv2.circle(mask,(int(round(x)),int(round(y))),12,0,-1)
        remaining=self.max_features-len(self.pixels)
        if remaining>0:
            new=cv2.goodFeaturesToTrack(gray,remaining,.01,12,mask=mask,blockSize=7)
            if new is not None:
                new=new.reshape(-1,2);ids=np.arange(self.next_id,self.next_id+len(new),dtype=np.int64);self.next_id+=len(new)
                self.pixels=np.vstack([self.pixels,new]);self.ids=np.r_[self.ids,ids]
        self.previous=gray
        points,covs=depth_at_pixels(self.pixels,cloud_body,Rbc,tbc,self.K,range_max)
        lidar_count=int(np.isfinite(points).all(axis=1).sum());triangulated=0
        if T_W_B is not None:
            R=T_W_B[:3,:3];t=T_W_B[:3,3];origin=R@tbc+t
            rays=np.linalg.solve(self.K,np.c_[self.pixels,np.ones(len(self.pixels))].T).T
            rays/=np.linalg.norm(rays,axis=1)[:,None];rays=rays@Rbc.T@R.T
            eligible=[]
            for i,(ident,ray) in enumerate(zip(self.ids,rays)):
                if ident not in self.anchors:self.anchors[ident]=(origin.copy(),ray.copy());continue
                if np.isfinite(points[i]).all():continue
                eligible.append(i)
            if eligible:
                indices=np.array(eligible);anchors=np.array([self.anchors[self.ids[i]][0] for i in indices]);first=np.array([self.anchors[self.ids[i]][1] for i in indices])
                world,Cworld=triangulate_many(anchors,first,origin,rays[indices],2./min(self.K[0,0],self.K[1,1]),range_max)
                valid=np.isfinite(world).all(axis=1);indices=indices[valid]
                points[indices]=(world[valid]-t)@R;covs[indices]=R.T@Cworld[valid]@R;triangulated=int(valid.sum())
            self.anchors={i:self.anchors[i] for i in self.ids}
        good=np.isfinite(points).all(axis=1)
        return self.ids[good],points[good],covs[good],dict(image_corners=len(self.pixels),lidar_depth=lidar_count,triangulated_depth=triangulated,depth_supported=int(good.sum()))


def triangulate(c0,d0,c1,d1,bearing_sigma,range_max):
    """Two observed rays with a measured estimator baseline, never a pose oracle."""
    p,C=triangulate_many(np.asarray(c0)[None],np.asarray(d0)[None],np.asarray(c1),np.asarray(d1)[None],bearing_sigma,range_max)
    return None if not np.isfinite(p[0]).all() else (p[0],C[0])


def triangulate_many(c0,d0,c1,d1,bearing_sigma,range_max):
    baseline=c1-c0;cosine=np.sum(d0*d1,axis=1);sine2=np.maximum(0.,1-cosine*cosine)
    b0=np.sum(d0*baseline,axis=1);b1=-np.sum(d1*baseline,axis=1)
    denominator=np.maximum(sine2,1e-12)
    depth0=(b0+cosine*b1)/denominator;depth=(cosine*b0+b1)/denominator
    a=c0+depth0[:,None]*d0;b=c1+depth[:,None]*d1
    miss=np.linalg.norm(a-b,axis=1)
    # Include angular observation uncertainty and a baseline-error floor.
    # This covariance scores views only; it is not exported to the estimators.
    sigma_depth=np.sqrt((2*(depth*bearing_sigma)**2+.05**2)/denominator)
    good=(np.linalg.norm(baseline,axis=1)>=.08)&(sine2>=.02**2)&(depth0>=.3)&(depth0<=range_max)&(depth>=.3)&(depth<=range_max)
    good&=(miss<=np.maximum(.04,depth*bearing_sigma*2))&(sigma_depth<=.45*depth)
    outer=d1[:,:,None]*d1[:,None,:]
    C=sigma_depth[:,None,None]**2*outer+(depth*bearing_sigma)[:,None,None]**2*(np.eye(3)-outer)
    p=(a+b)*.5;p[~good]=np.nan;C[~good]=np.nan
    return p,C
