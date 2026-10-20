// Actual sockets and multipart payloads, with repeated OS signals interrupting
// a blocked sender. The receiver requires every part in exact sequence.
#include <zmq.h>
#include <dlfcn.h>
#include <pthread.h>
#include <csignal>
#include <thread>
#include <atomic>
#include <cassert>
#include <cstring>
#include <iostream>
#include <chrono>
int main(){
  auto retry=reinterpret_cast<unsigned long long(*)()>(dlsym(RTLD_DEFAULT,"uas_zmq_eintr_retries"));assert(retry);
  struct sigaction action{};action.sa_handler=[](int){};sigemptyset(&action.sa_mask);sigaction(SIGUSR1,&action,nullptr);
  void* context=zmq_ctx_new();void* sender=zmq_socket(context,ZMQ_PUSH);int hwm=1;zmq_setsockopt(sender,ZMQ_SNDHWM,&hwm,sizeof(hwm));
  assert(zmq_bind(sender,"inproc://retry_test")==0);std::atomic<bool> done{false};const auto main=pthread_self();
  std::thread signals([&](){while(!done){pthread_kill(main,SIGUSR1);std::this_thread::sleep_for(std::chrono::microseconds(100));}});
  std::thread receiver([&](){void* sock=zmq_socket(context,ZMQ_PULL);zmq_setsockopt(sock,ZMQ_RCVHWM,&hwm,sizeof(hwm));assert(zmq_connect(sock,"inproc://retry_test")==0);
    for(int i=0;i<2000;++i){for(int j=0;j<4;++j){int value=-1;assert(zmq_recv(sock,&value,sizeof(value),0)==sizeof(value));assert(value==4*i+j);int more=0;size_t size=sizeof(more);assert(zmq_getsockopt(sock,ZMQ_RCVMORE,&more,&size)==0);assert(bool(more)==(j<3));}std::this_thread::sleep_for(std::chrono::microseconds(250));}zmq_close(sock);});
  for(int i=0;i<2000;++i)for(int j=0;j<4;++j){zmq_msg_t msg;assert(zmq_msg_init_size(&msg,sizeof(int))==0);int value=4*i+j;std::memcpy(zmq_msg_data(&msg),&value,sizeof(value));assert(zmq_msg_send(&msg,sender,j<3?ZMQ_SNDMORE:0)==sizeof(value));zmq_msg_close(&msg);}
  receiver.join();done=true;signals.join();zmq_close(sender);zmq_ctx_term(context);
  assert(retry()>0);std::cout<<"2000 multipart messages, 8000 exact parts, retries="<<retry()<<", no loss/duplication\n";
}
