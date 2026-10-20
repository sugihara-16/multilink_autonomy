#include "ordered_imu_prefix.hpp"
#include <deque>
#include <cassert>
int main(){
  const auto stamp=[](uint64_t n){return n;};
  std::deque<uint64_t> q{2000000,3000000,4000000};
  assert(orderedImuPrefix(q,1000000,3000000,stamp)==2);
  assert(orderedImuPrefix(q,1000000,2999999,stamp)==2);
  assert(orderedImuPrefix(q,1000000,3000001,stamp)==3);
  assert(orderedImuPrefix(q,0,1999999,stamp)==0);
  assert(orderedImuPrefix(q,3000000,2999999,stamp)==0);
  bool threw=false;try{orderedImuPrefix(q,1000000,4000001,stamp);}catch(const std::runtime_error&){threw=true;}
  assert(threw);
}
