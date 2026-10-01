#include <ros/ros.h>
#include <grid_map_filters/NormalVectorsFilter.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
using Base = filters::FilterBase<grid_map::GridMap>;

grid_map::GridMap run(const grid_map::GridMap& input, const std::string& algorithm, bool parallel, int threads) {
  grid_map::NormalVectorsFilter filter;
  XmlRpc::XmlRpcValue config, params;
  config["name"] = "normals";
  config["type"] = "gridMapFilters/NormalVectorsFilter";
  params["algorithm"] = algorithm;
  params["radius"] = 0.3;
  params["parallelization_enabled"] = parallel;
  params["thread_number"] = threads;
  params["normal_vector_positive_axis"] = "z";
  params["input_layer"] = "elevation";
  params["output_layers_prefix"] = "normal_";
  config["params"] = params;
  if (!filter.Base::configure(config)) throw std::runtime_error("configure failed");
  grid_map::GridMap output;
  if (!filter.update(input, output)) throw std::runtime_error("update failed");
  return output;
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "normal_filter_regression", ros::init_options::NoSigintHandler);
  ros::Time::init();
  grid_map::GridMap input({"elevation"});
  input.setGeometry(grid_map::Length(2., 2.), 0.1);
  input.setFrameId("map");
  for (grid_map::GridMapIterator it(input); !it.isPastEnd(); ++it) {
    grid_map::Position p;
    input.getPosition(*it, p);
    input.at("elevation", *it) = 0.2*p.x() - 0.1*p.y();
  }
  input["elevation"].topLeftCorner(3, 3).setConstant(NAN);
  for (const std::string algorithm : {"area", "raster"}) {
    const auto serial = run(input, algorithm, false, 1);
    for (const int threads : {-1, 1, 2}) {
      const auto parallel = run(input, algorithm, true, threads);
      double max_error = 0;
      int count = 0;
      for (const std::string axis : {"x", "y", "z"}) {
        const auto& a = serial["normal_" + axis];
        const auto& b = parallel["normal_" + axis];
        for (int i = 0; i < a.size(); ++i) {
          if (std::isnan(a.data()[i]) && std::isnan(b.data()[i])) continue;
          if (!std::isfinite(a.data()[i]) || !std::isfinite(b.data()[i])) return 2;
          max_error = std::max(max_error, std::abs(double(a.data()[i] - b.data()[i])));
          ++count;
        }
      }
      std::cout << algorithm << " threads=" << threads << " finite_components=" << count << " max_error=" << max_error << std::endl;
      if (count < 300 || max_error > 1e-6) return 3;
    }
  }
}
