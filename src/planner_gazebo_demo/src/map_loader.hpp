// map_loader.hpp
//
// Utility to load a PGM + YAML map pair and provide grid-based collision
// checking for the planner nodes.

#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "yaml-cpp/yaml.h"

struct MapInfo
{
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;

  int width = 0;
  int height = 0;

  double occupied_thresh = 0.65;
  double free_thresh = 0.25;

  int negate = 0;

  std::string mode = "trinary";
  std::string image;
};

class MapLoader
{
public:
  bool load(const std::string & yaml_path)
  {
    YAML::Node config;

    try {
      config = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception &) {
      return false;
    }

    if (!config["resolution"] || !config["origin"] || !config["image"]) {
      return false;
    }

    info_.resolution = config["resolution"].as<double>();

    auto origin = config["origin"].as<std::vector<double>>();
    if (origin.size() < 2) {
      return false;
    }

    info_.origin_x = origin[0];
    info_.origin_y = origin[1];
    info_.origin_yaw = origin.size() > 2 ? origin[2] : 0.0;

    info_.image = config["image"].as<std::string>();
    info_.mode = config["mode"] ? config["mode"].as<std::string>() : "trinary";
    info_.negate = config["negate"] ? config["negate"].as<int>() : 0;

    info_.occupied_thresh = config["occupied_thresh"] ?
      config["occupied_thresh"].as<double>() : 0.65;

    info_.free_thresh = config["free_thresh"] ?
      config["free_thresh"].as<double>() : 0.25;

    std::string pgm_path = resolveImagePath(yaml_path, info_.image);

    if (!loadPgm(pgm_path)) {
      return false;
    }

    buildOccupancyGrid();
    return true;
  }

  const nav_msgs::msg::OccupancyGrid & getOccupancyGrid() const
  {
    return grid_;
  }

  const MapInfo & getInfo() const
  {
    return info_;
  }

  bool worldToGrid(double wx, double wy, int & gx, int & gy) const
  {
    gx = static_cast<int>((wx - info_.origin_x) / info_.resolution);
    gy = static_cast<int>((wy - info_.origin_y) / info_.resolution);

    return gx >= 0 && gx < info_.width && gy >= 0 && gy < info_.height;
  }

  void gridToWorld(int gx, int gy, double & wx, double & wy) const
  {
    wx = info_.origin_x + (static_cast<double>(gx) + 0.5) * info_.resolution;
    wy = info_.origin_y + (static_cast<double>(gy) + 0.5) * info_.resolution;
  }

  bool isOccupied(double wx, double wy, double margin = 0.0) const
  {
    if (margin <= 1e-6) {
      int gx, gy;
      if (!worldToGrid(wx, wy, gx, gy)) {
        return true;
      }

      const int idx = gy * info_.width + gx;
      const int occ = grid_.data[idx];

      // Safety policy:
      // occupied = obstacle
      // unknown = obstacle
      return occ < 0 || occ > 50;
    }

    int cx, cy;
    if (!worldToGrid(wx, wy, cx, cy)) {
      return true;
    }

    const int mr = static_cast<int>(std::ceil(margin / info_.resolution));

    for (int dx = -mr; dx <= mr; ++dx) {
      for (int dy = -mr; dy <= mr; ++dy) {
        const int gx = cx + dx;
        const int gy = cy + dy;

        if (gx < 0 || gx >= info_.width || gy < 0 || gy >= info_.height) {
          return true;
        }

        const int occ = grid_.data[gy * info_.width + gx];

        if (occ < 0 || occ > 50) {
          double cell_wx, cell_wy;
          gridToWorld(gx, gy, cell_wx, cell_wy);

          if (std::hypot(cell_wx - wx, cell_wy - wy) <= margin) {
            return true;
          }
        }
      }
    }

    return false;
  }

  bool isFree(double wx, double wy, double margin = 0.0) const
  {
    if (margin > 1e-6) {
      return !isOccupied(wx, wy, margin);
    }

    int gx, gy;
    if (!worldToGrid(wx, wy, gx, gy)) {
      return false;
    }

    const int idx = gy * info_.width + gx;
    return grid_.data[idx] == 0;
  }

private:
  std::string resolveImagePath(
    const std::string & yaml_path,
    const std::string & image_path) const
  {
    if (!image_path.empty() && image_path[0] == '/') {
      return image_path;
    }

    const auto pos = yaml_path.find_last_of("/");
    if (pos == std::string::npos) {
      return image_path;
    }

    const std::string dir = yaml_path.substr(0, pos);
    return dir + "/" + image_path;
  }

  bool readPgmToken(std::istream & in, std::string & token)
  {
    token.clear();

    char c;

    while (in.get(c)) {
      if (std::isspace(static_cast<unsigned char>(c))) {
        continue;
      }

      if (c == '#') {
        std::string dummy;
        std::getline(in, dummy);
        continue;
      }

      token.push_back(c);
      break;
    }

    while (in.get(c)) {
      if (std::isspace(static_cast<unsigned char>(c))) {
        break;
      }

      if (c == '#') {
        std::string dummy;
        std::getline(in, dummy);
        break;
      }

      token.push_back(c);
    }

    return !token.empty();
  }

  bool loadPgm(const std::string & path)
  {
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) {
      return false;
    }

    std::string token;

    if (!readPgmToken(f, token)) {
      return false;
    }

    if (token != "P5") {
      return false;
    }

    if (!readPgmToken(f, token)) {
      return false;
    }
    info_.width = std::stoi(token);

    if (!readPgmToken(f, token)) {
      return false;
    }
    info_.height = std::stoi(token);

    if (!readPgmToken(f, token)) {
      return false;
    }
    const int maxval = std::stoi(token);

    if (maxval <= 0 || maxval > 255) {
      return false;
    }

    raw_data_.resize(static_cast<size_t>(info_.width * info_.height));

    f.read(
      reinterpret_cast<char *>(raw_data_.data()),
      static_cast<std::streamsize>(raw_data_.size()));

    return static_cast<size_t>(f.gcount()) == raw_data_.size();
  }

  int pixelToOccupancy(uint8_t val) const
  {
    // nav2 map_saver trinary output usually uses:
    // 0   = occupied
    // 254 = free
    // 205 = unknown
    if (info_.mode == "trinary") {
      if (val <= 10) {
        return 100;
      }

      if (val >= 250) {
        return 0;
      }

      return -1;
    }

    double occ_prob;

    if (info_.negate) {
      occ_prob = static_cast<double>(val) / 255.0;
    } else {
      occ_prob = 1.0 - static_cast<double>(val) / 255.0;
    }

    if (occ_prob >= info_.occupied_thresh) {
      return 100;
    }

    if (occ_prob <= info_.free_thresh) {
      return 0;
    }

    return -1;
  }

  void buildOccupancyGrid()
  {
    grid_.header.frame_id = "map";

    grid_.info.resolution = info_.resolution;
    grid_.info.width = info_.width;
    grid_.info.height = info_.height;

    grid_.info.origin.position.x = info_.origin_x;
    grid_.info.origin.position.y = info_.origin_y;
    grid_.info.origin.position.z = 0.0;
    grid_.info.origin.orientation.w = 1.0;

    grid_.data.resize(static_cast<size_t>(info_.width * info_.height));

    for (int gy = 0; gy < info_.height; ++gy) {
      for (int gx = 0; gx < info_.width; ++gx) {
        // PGM image row 0 is top.
        // OccupancyGrid row 0 is bottom.
        const int image_y = info_.height - 1 - gy;

        const size_t image_idx =
          static_cast<size_t>(image_y * info_.width + gx);

        const size_t grid_idx =
          static_cast<size_t>(gy * info_.width + gx);

        grid_.data[grid_idx] = static_cast<int8_t>(
          pixelToOccupancy(raw_data_[image_idx]));
      }
    }
  }

  MapInfo info_;
  std::vector<uint8_t> raw_data_;
  nav_msgs::msg::OccupancyGrid grid_;
};