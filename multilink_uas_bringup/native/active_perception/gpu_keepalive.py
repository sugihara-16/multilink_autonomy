#!/usr/bin/env python3
"""Optional bounded GPU clock experiment; releases its CUDA context on exit."""
import ctypes as c,time,signal,argparse
p=argparse.ArgumentParser();p.add_argument("--duration",type=float,default=60);p.add_argument("--blocks",type=int,default=1);p.add_argument("--interval",type=float,default=.005);p.add_argument('--cycles',type=int,default=1000000);a=p.parse_args()
cu=c.CDLL('libcuda.so.1')
def call(n,*a):
 r=getattr(cu,n)(*a)
 if r:raise RuntimeError(n+': '+str(r))
call('cuInit',0);dev=c.c_int();call('cuDeviceGet',c.byref(dev),0)
ctx=c.c_void_p();# Block the CPU while the CUDA pulse runs; preserve the GPU work and interval.
call('cuCtxCreate_v2',c.byref(ctx),4,dev)
ptx=b'''\n.version 6.0\n.target sm_50\n.address_size 64\n.visible .entry pulse(.param .u64 cycles) {\n.reg .pred p;\n.reg .b64 start,now,elapsed,n;\nld.param.u64 n,[cycles];\nmov.u64 start, %clock64;\nloop:\nmov.u64 now, %clock64;\nsub.u64 elapsed,now,start;\nsetp.lt.u64 p,elapsed,n;\n@p bra loop;\nret;\n}\n'''
module=c.c_void_p();call('cuModuleLoadData',c.byref(module),c.c_char_p(ptx));f=c.c_void_p();call('cuModuleGetFunction',c.byref(f),module,b'pulse')
cycles=c.c_uint64(a.cycles);params=(c.c_void_p*1)(c.cast(c.pointer(cycles),c.c_void_p))
end=time.monotonic()+a.duration if a.duration>0 else float("inf")
print('GPU memory-clock keepalive; no power/clock settings modified',flush=True)
try:
 while time.monotonic()<end:
  call('cuLaunchKernel',f,a.blocks,1,1,32,1,1,0,None,params,None);call('cuCtxSynchronize');time.sleep(a.interval)
except KeyboardInterrupt:pass
finally:call('cuCtxDestroy_v2',ctx)
