"""Multi-joint view roadmap and observed-landmark information objective.

No environment labels, route or ground truth enter the view search. Mechanical
limits and sampled self-clearance constrain nodes AND connecting joint paths.
"""
import heapq
import itertools
import numpy as np
from scipy.spatial import cKDTree
from scipy.stats import qmc
from core import skew


def interpolate_path(a,b,step=np.deg2rad(3.)):
    n=max(2,int(np.ceil(np.max(abs(np.asarray(b)-a))/step))+1)
    return np.linspace(a,b,n)


class Roadmap:
    def __init__(self,kin):
        self.kin=kin
        samples=[kin.home.copy()]
        # Structured folded shapes complement full-limit low-discrepancy samples.
        for yaw in itertools.product([0.,30.,60.,89.38],repeat=3):
            for pitch in ([0,0,0],[30,-30,0],[-30,30,0],[0,30,-30],[0,-30,30],[30,0,-30],[-30,0,30]):
                q=np.empty(6);q[::2]=np.deg2rad(pitch);q[1::2]=np.deg2rad(yaw)
                if kin.valid(q):samples.append(q)
        sobol=qmc.Sobol(6,scramble=False).random_base2(10)
        samples.extend(q for q in kin.lower+sobol*(kin.upper-kin.lower) if kin.valid(q))
        # Preserve direction diversity, favoring less deformation within a bin.
        cells={}
        for q in samples:
            T=kin.camera(q);d=T[:3,2]
            key=(int(np.floor(np.arctan2(d[1],d[0])/np.deg2rad(15))),int(np.floor(np.arcsin(np.clip(d[2],-1,1))/np.deg2rad(15))))
            cost=np.linalg.norm(q-kin.home)
            if key not in cells or cost<cells[key][0]:cells[key]=(cost,q)
        self.q=np.array([kin.home]+[x[1] for x in sorted(cells.values(),key=lambda x:x[0]) if np.linalg.norm(x[1]-kin.home)>1e-6])
        self.transforms=np.array([kin.camera(q) for q in self.q]);self.edges={};self.neighbors={}
        _,nn=cKDTree(self.q).query(self.q,k=min(12,len(self.q)))
        for i,row in enumerate(nn):
            self.neighbors.setdefault(i,set())
            for j in row[1:]:self.neighbors[i].add(int(j));self.neighbors.setdefault(int(j),set()).add(i)
        self.home_paths=self.paths(kin.home)

    def edge(self,i,j):
        k=tuple(sorted((i,j)))
        if k not in self.edges:self.edges[k]=self.kin.path_valid(self.q[i],self.q[j],np.deg2rad(5))
        return self.edges[k]

    def paths(self,current):
        costs={};parent={};queue=[]
        for i in np.argsort(np.linalg.norm(self.q-current,axis=1))[:12]:
            if self.kin.path_valid(current,self.q[i],np.deg2rad(3)):
                cost=float(np.max(abs(self.q[i]-current))/self.kin.speed)
                costs[int(i)]=cost;parent[int(i)]=None;heapq.heappush(queue,(cost,int(i)))
        while queue:
            cost,i=heapq.heappop(queue)
            if cost!=costs[i]:continue
            for j in self.neighbors[i]:
                new=cost+float(np.max(abs(self.q[i]-self.q[j]))/self.kin.speed)
                if new>=costs.get(j,np.inf) or not self.edge(i,j):continue
                costs[j]=new;parent[j]=i;heapq.heappush(queue,(new,j))
        result={}
        for i,cost in costs.items():
            chain=[];j=i
            while j is not None:chain.append(self.q[j]);j=parent[j]
            result[i]=(cost,[np.array(current)]+chain[::-1])
        return result

    def audit(self):
        ids=list(self.home_paths);d=self.transforms[ids,:3,2]
        return dict(nodes=len(self.q),reachable=len(ids),azimuth_deg=np.rad2deg(np.arctan2(d[:,1],d[:,0])).tolist(),
                    elevation_deg=np.rad2deg(np.arcsin(np.clip(d[:,2],-1,1))).tolist(),
                    joint_min_deg=np.rad2deg(self.kin.lower).tolist(),joint_max_deg=np.rad2deg(self.kin.upper).tolist(),
                    clearance='sampled body-tube and rotor-module bounds; not exact meshes',
                    support='minimum planar spread 0.10m, vertical rotor span <=1.25m')


def visual_information(points,covariances,T,bearing_sigma):
    """Independent map-landmark proxy, including camera translation lever arm.

    Map uncertainty is included, but inter-landmark and shared-IMU correlations
    are not claimed to be exact. This matrix is for view selection only.
    """
    rays=(points-T[:3,3])@T[:3,:3];d=np.linalg.norm(rays,axis=1)
    u=rays/np.maximum(d[:,None],1e-6)
    P=np.eye(3)-u[:,:,None]*u[:,None,:]
    J=np.array([np.column_stack((skew(p),-np.eye(3))) for p in points])
    B=P@T[:3,:3].T/np.maximum(d[:,None,None],1e-6)
    J=B@J
    S=B@covariances@B.transpose(0,2,1)+bearing_sigma**2*np.eye(3)
    return J.transpose(0,2,1)@np.linalg.solve(S,J)


def view_weights(points,T,cfg,views=None,ages=None):
    r=points-T[:3,3];p=r@T[:3,:3]
    edge=np.maximum(abs(p[:,0])/np.maximum(p[:,2],1e-6)/cfg['tan_half_hfov'],abs(p[:,1])/np.maximum(p[:,2],1e-6)/cfg['tan_half_vfov'])
    w=np.clip((1-edge)/.2,0,1)*(p[:,2]>.1)
    if views is not None:
        unit=r/np.maximum(np.linalg.norm(r,axis=1)[:,None],1e-6)
        w*=np.clip((np.sum(unit*views,axis=1)-.5)/.5,0,1)
    if ages is not None:w*=np.exp(-np.maximum(ages,0)/30.)
    return w


def position_logdet(H):
    H=(H+H.T)*.5
    S=H[3:,3:]-H[3:,:3]@np.linalg.solve(H[:3,:3],H[:3,3:])
    return np.linalg.slogdet((S+S.T)*.5)[1]


def information_gain(H,points,C,T,weights,cfg):
    contributions=visual_information(points,C,T,cfg['bearing_sigma'])
    return float(position_logdet(H+np.einsum('n,nij->ij',weights,contributions))-position_logdet(H))
