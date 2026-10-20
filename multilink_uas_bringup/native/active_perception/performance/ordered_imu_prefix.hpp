#pragma once
#include <cstddef>
#include <cstdint>
#include <stdexcept>

// Canonical prefix needed to interpolate at a sensor timestamp. Never feed
// arbitrary future buffered measurements; at most the first bracketing IMU.
template<class Queue,class Stamp>
size_t orderedImuPrefix(const Queue& queue,uint64_t processed,uint64_t event,Stamp stamp) {
  if(processed>=event)return 0;
  if(processed==0 && !queue.empty() && stamp(queue.front())>event)return 0;
  size_t n=0;
  for(const auto& message:queue){++n;if(stamp(message)>=event)return n;}
  throw std::runtime_error("Ordered IMU watermark did not bracket event");
}
