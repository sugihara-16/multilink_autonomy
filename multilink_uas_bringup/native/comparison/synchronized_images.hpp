#pragma once
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <vector>
// Exact acquisition timestamps: never substitute an older image for a missing camera.
template<class Image> class SynchronizedImages {
 public:
  struct Bundle {int64_t stamp; std::vector<Image> images;};
  SynchronizedImages(size_t cameras,size_t capacity):n_(cameras),capacity_(capacity) {
    if(!n_ || !capacity_)throw std::invalid_argument("Camera count and capacity must be positive");
  }
  std::optional<Bundle> push(size_t camera,int64_t stamp,const Image& image) {
    if(camera>=n_)throw std::out_of_range("camera index");
    if(stamp<=last_){++late;return {};}
    auto& frame=pending_.try_emplace(stamp,n_).first->second;
    if(frame[camera]){++duplicates;return {};}
    frame[camera]=image;
    bool complete=true;for(const auto& x:frame)complete&=x.has_value();
    if(complete){
      Bundle result{stamp,{}};for(const auto& x:frame)result.images.push_back(*x);
      for(auto it=pending_.begin();it!=pending_.end() && it->first<=stamp;){if(it->first!=stamp)++dropped;it=pending_.erase(it);}
      last_=stamp;return result;
    }
    while(pending_.size()>capacity_){pending_.erase(pending_.begin());++dropped;}
    return {};
  }
  size_t pending()const{return pending_.size();}
  uint64_t duplicates=0,late=0,dropped=0;
 private:
  size_t n_,capacity_;int64_t last_=-1;
  std::map<int64_t,std::vector<std::optional<Image>>> pending_;
};
