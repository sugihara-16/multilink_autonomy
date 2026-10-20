#pragma once

// An alternative index for the existing gtsam_points iVox. The point storage,
// insertion order, neighbour order, LRU policy and snapshot sharing are unchanged.
// Only the coordinate -> voxel-id hash table is contiguous instead of node based.
#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/ann/knn_result.hpp>
#include <gtsam_points/util/fast_floor.hpp>
#include <algorithm>
#include <atomic>
#include <array>
#include <limits>
#include <stdexcept>
#include <vector>

namespace uas_performance {
class CompactIVox final : public gtsam_points::iVox {
  struct Entry {
    int x=0, y=0, z=0;
    size_t id=empty;
    static constexpr size_t empty=std::numeric_limits<size_t>::max();
  };
  std::vector<Entry> index_;
  size_t count_=0;
  inline static std::atomic<size_t> next_token_{1};
  size_t token_=next_token_.fetch_add(1,std::memory_order_relaxed);

  size_t slot(const Eigen::Vector3i& c) const {
    size_t i=gtsam_points::XORVector3iHash{}(c)&(index_.size()-1);
    while(index_[i].id!=Entry::empty &&
          (index_[i].x!=c.x() || index_[i].y!=c.y() || index_[i].z!=c.z()))
      i=(i+1)&(index_.size()-1);
    return i;
  }
  void rehash(size_t capacity) {
    // Keep a power of two and load <= 1/2, with at least one empty slot.
    std::vector<Entry> previous=std::move(index_);
    index_.assign(capacity,Entry{});count_=0;
    for(const auto& e:previous) if(e.id!=Entry::empty) put({e.x,e.y,e.z},e.id);
  }
  void put(const Eigen::Vector3i& c,size_t id) {
    if(index_.empty()) rehash(16);
    if((count_+1)*2>=index_.size()) rehash(index_.size()*2);
    auto& e=index_[slot(c)];
    if(e.id==Entry::empty) ++count_;
    e={c.x(),c.y(),c.z(),id};
  }
  size_t find(const Eigen::Vector3i& c) const {
    return index_.empty()?Entry::empty:index_[slot(c)].id;
  }
public:
  explicit CompactIVox(double leaf):gtsam_points::iVox(leaf) {}
  CompactIVox(const CompactIVox& other):gtsam_points::iVox(other),index_(other.index_),count_(other.count_) {}
  CompactIVox& operator=(const CompactIVox& other) {
    if(this!=&other){gtsam_points::iVox::operator=(other);index_=other.index_;count_=other.count_;token_=next_token_.fetch_add(1,std::memory_order_relaxed);}return *this;
  }
  void clear() override {
    token_=next_token_.fetch_add(1,std::memory_order_relaxed);
    gtsam_points::iVox::clear();index_.clear();count_=0;
  }
  void insert(const gtsam_points::PointCloud& cloud) override {
    token_=next_token_.fetch_add(1,std::memory_order_relaxed);
    for(size_t i=0;i<cloud.size();++i) {
      const Eigen::Vector3i c=gtsam_points::fast_floor(cloud.points[i]*inv_leaf_size).head<3>();
      size_t id=find(c);
      if(id==Entry::empty) {
        id=flat_voxels.size();
        auto v=std::make_shared<std::pair<gtsam_points::VoxelInfo,gtsam_points::FlatContainer>>(
          gtsam_points::VoxelInfo(c,lru_counter),gtsam_points::FlatContainer());
        put(c,id);flat_voxels.emplace_back(std::move(v));
      }
      auto& [info,voxel]=*flat_voxels[id];
      info.lru=lru_counter;voxel.add(voxel_setting,cloud,i);
    }
    if((++lru_counter)%lru_clear_cycle==0) {
      auto end=std::remove_if(flat_voxels.begin(),flat_voxels.end(),[&](const auto& v){
        return v->first.lru+lru_horizon<lru_counter;
      });
      if(end!=flat_voxels.end()) {
        flat_voxels.erase(end,flat_voxels.end());
        std::fill(index_.begin(),index_.end(),Entry{});count_=0;
        for(size_t i=0;i<flat_voxels.size();++i) put(flat_voxels[i]->first.coord,i);
      }
    }
    // FlatContainer::finalize is empty, so no whole-map traversal is needed.
  }
  template<int N> size_t search(const double* pt,size_t k,size_t* ids,double* distances,
      double max_distance) const {
    const Eigen::Vector4d query=(Eigen::Vector4d()<<pt[0],pt[1],pt[2],1.).finished();
    const Eigen::Vector3i center=gtsam_points::fast_floor(query*inv_leaf_size).head<3>();
    size_t voxel_index=0;
    const auto transform=[&](size_t point_index){return calc_index(voxel_index,point_index);};
    gtsam_points::KnnResult<N,decltype(transform)> result(ids,distances,N>0?-1:static_cast<int>(k),transform,max_distance);
#if UAS_IVOX_NEIGHBOR_CACHE
    // Cache only index lookups, never point values or distances. A fresh token
    // on copy/insert/clear prevents stale IDs and address-reuse aliasing.
    struct Neighborhood {size_t token=0,count=0;Eigen::Vector3i cell;std::array<size_t,27> ids;};
    thread_local std::array<Neighborhood,512> neighborhoods;
    auto& neighbors=neighborhoods[gtsam_points::XORVector3iHash{}(center)&511];
    if(neighbors.token!=token_ || neighbors.count!=offsets.size() || neighbors.cell!=center) {
      neighbors.token=token_;neighbors.cell=center;neighbors.count=offsets.size();
      for(size_t i=0;i<offsets.size();++i)neighbors.ids[i]=find(center+offsets[i]);
    }
#if UAS_IVOX_PREFETCH
    for(size_t i=0;i<offsets.size();++i)if(neighbors.ids[i]!=Entry::empty)
      __builtin_prefetch(flat_voxels[neighbors.ids[i]].get(),0,1);
    for(size_t i=0;i<offsets.size();++i)if(neighbors.ids[i]!=Entry::empty)
      __builtin_prefetch(flat_voxels[neighbors.ids[i]]->second.points.data(),0,1);
#endif
    size_t offset_index=0;
#endif
    for(const auto& offset:offsets) {
#if UAS_IVOX_NEIGHBOR_CACHE
      const size_t cached_id=neighbors.ids[offset_index++];
#endif
#if UAS_IVOX_AABB
      // Conservative squared distance to the entire cell. If even its closest
      // possible point cannot enter the current result, skip that cell. Keep
      // original visitation/tie order for all cells that are not excluded.
      const Eigen::Vector3i cell=center+offset;
      double lower_bound=0.;
      for(int axis=0;axis<3;++axis) {
        const double lo=double(cell[axis])/inv_leaf_size,hi=(double(cell[axis])+1.)/inv_leaf_size;
        const double margin=32.*std::numeric_limits<double>::epsilon()*(1.+std::abs(pt[axis])+std::abs(lo)+std::abs(hi));
        const double d=std::max(0.,std::max(lo-pt[axis],pt[axis]-hi)-margin);
        lower_bound+=d*d;
      }
      if(lower_bound>result.worst_distance()*(1.+32.*std::numeric_limits<double>::epsilon()))continue;
#endif
#if UAS_IVOX_NEIGHBOR_CACHE
      const size_t id=cached_id;
#else
      const size_t id=find(center+offset);
#endif
      if(id==Entry::empty)continue;
      voxel_index=id;flat_voxels[id]->second.knn_search(query,result);
    }
    return result.num_found();
  }
  size_t knn_search(const double* pt,size_t k,size_t* ids,double* distances,
      double max_distance=std::numeric_limits<double>::max()) const override {
#if UAS_IVOX_STATIC5
    if(k==5)return search<5>(pt,k,ids,distances,max_distance);
#endif
    return search<-1>(pt,k,ids,distances,max_distance);
  }
};
} // namespace uas_performance
