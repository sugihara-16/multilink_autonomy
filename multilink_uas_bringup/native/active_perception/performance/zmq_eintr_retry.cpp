// Retry the SAME ZeroMQ message part after EINTR. Never interpolate a sensor
// sample, restart a multipart message, or retry other errors (EAGAIN/ETERM).
// ZeroMQ documents that on failure the message is intact and not sent:
// https://libzmq.readthedocs.io/en/latest/zmq_msg_send.html
#include <zmq.h>
#include <dlfcn.h>
#include <cerrno>
#include <atomic>
#include <cstdio>
static std::atomic<unsigned long long> retries{0};
extern "C" unsigned long long uas_zmq_eintr_retries(){return retries.load();}
extern "C" int zmq_msg_send(zmq_msg_t* message,void* socket,int flags){
  using Send=int(*)(zmq_msg_t*,void*,int);
  static Send original=reinterpret_cast<Send>(dlsym(RTLD_NEXT,"zmq_msg_send"));
  if(!original){errno=ENOSYS;return -1;}
  for(;;){
    const int result=original(message,socket,flags);
    if(result>=0 || errno!=EINTR)return result;
    ++retries;
  }
}
__attribute__((destructor)) static void report(){
  if(retries.load())std::fprintf(stderr,"[uas transport] retried %llu interrupted message parts; original payload and stamp preserved\n",retries.load());
}
