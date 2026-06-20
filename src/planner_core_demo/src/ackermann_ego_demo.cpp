// ackermann_ego_demo.cpp
//
// Task 15-③ Ackermann variant: compare ESDF-based vs EGO (ESDF-free) back-end.
//
// Runs the same front-end path through both back-ends and reports timing,
// smoothness, curvature and speed-up.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

struct Vec3 { double x; double y; double z; };
struct BoxObstacle { double x; double y; double z; double sx; double sy; double sz; };
struct AckermannState { double x, y, theta, v; };
struct KinoNode { AckermannState s; double time, g_score, f_score; int parent; long long key; };
struct QueueItem { double f_score; int node_id; bool operator<(const QueueItem & o) const { return f_score > o.f_score; } };

struct BackendResult
{
  std::vector<geometry_msgs::msg::Point> path;
  double build_ms = 0.0, opt_ms = 0.0, total_ms = 0.0;
  double smooth = 0.0, curv = 0.0, len = 0.0;
};

class AckermannEgoDemoNode : public rclcpp::Node
{
public:
  AckermannEgoDemoNode() : Node("ackermann_ego_demo_node")
  {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planner_core_demo/markers", 10);
    initParams();
    generateObstacles();
    if (runKinoAStar()) {
      generateControlPoints();
      esdf_result_ = optimize(true);
      ego_result_  = optimize(false);
      report();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Front-end failed.");
    }
    timer_ = this->create_wall_timer(500ms, std::bind(&AckermannEgoDemoNode::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "ackermann_ego_demo_node started.");
  }

private:
  void initParams()
  {
    x_min_ = -8.0; x_max_ = 8.0; y_min_ = -7.0; y_max_ = 7.0; z0_ = 1.0;
    pos_res_ = 0.4; theta_res_ = M_PI / 12.0; vel_res_ = 0.5;
    nx_ = static_cast<int>((x_max_ - x_min_) / pos_res_);
    ny_ = static_cast<int>((y_max_ - y_min_) / pos_res_);
    ntheta_ = static_cast<int>(2.0 * M_PI / theta_res_);
    wheelbase_ = 1.5; max_vel_ = 2.0; max_steer_ = 35.0 * M_PI / 180.0;
    kappa_max_ = std::tan(max_steer_) / wheelbase_;
    v_cruise_ = 1.5; dt_ = 0.4; primitive_check_ = 8;
    goal_tol_ = 0.65; goal_theta_tol_ = M_PI / 6.0; safety_ = 0.22;
    max_time_ = 25.0; max_expand_ = 200000;
    start_state_.x = -7.0; start_state_.y = -6.0;
    start_state_.theta = std::atan2(6.0, 7.0); start_state_.v = 0.0;
    goal_x_ = 7.0; goal_y_ = 6.0;
    nv_ = static_cast<int>(2.0 * max_vel_ / vel_res_) + 1;
    scenario_ = this->declare_parameter("scenario", std::string("dense"));

    bspline_seg_ = 12; opt_iters_ = 300; opt_step_ = 0.06;
    lambda_s_ = 1.0; lambda_c_ = 2.2; lambda_k_ = 2.0; d_safe_ = 0.55;
    esdf_res_ = 0.12;
  }

  void generateObstacles()
  {
    obstacles_.clear(); std::mt19937 rng(12);
    auto tooClose = [&](double ox, double oy) {
      return std::hypot(ox - start_state_.x, oy - start_state_.y) < 2.0 ||
             std::hypot(ox - goal_x_, oy - goal_y_) < 2.0;
    };
    const std::string sc = scenario_;
    if (sc == "narrow") {
      const double wz = 1.5, wh = 3.0, ws = 5.5, wt = 0.35, gh = 1.2;
      auto aw = [&](double cy, double gx) {
        obstacles_.push_back(BoxObstacle{-ws - 0.1, cy, wz, ws - gh + 0.1, wt, wh});
        obstacles_.push_back(BoxObstacle{ gx + gh, cy, wz, ws - gh + 0.2, wt, wh});
      };
      aw(-1.5, -2.0); aw(0.0, 1.5); aw(1.3, -1.0);
      std::uniform_real_distribution<double> px(-6.5, 6.5), py(-5.8, 5.8), sz(0.4, 0.7), h(0.8, 2.2);
      for (int i = 0; i < 35; ++i) {
        BoxObstacle obs; obs.x = px(rng); obs.y = py(rng);
        obs.sx = sz(rng); obs.sy = sz(rng); obs.sz = h(rng); obs.z = obs.sz / 2.0;
        if (tooClose(obs.x, obs.y)) continue;
        bool blocked = false;
        for (const auto & e : obstacles_)
          if (std::abs(obs.x - e.x) < e.sx / 2.0 + obs.sx / 2.0 + 0.3 &&
              std::abs(obs.y - e.y) < e.sy / 2.0 + obs.sy / 2.0 + 0.3) { blocked = true; break; }
        if (!blocked) obstacles_.push_back(obs);
      }
    } else if (sc == "clustered") {
      struct { double cx; double cy; int n; } cls[] = {{-2, -2, 18}, {2, 1.5, 20}, {-0.5, 3.5, 16}};
      for (const auto & cl : cls) {
        std::normal_distribution<double> cx(cl.cx, 2.0), cy(cl.cy, 1.8);
        std::uniform_real_distribution<double> sz(0.5, 0.95), h(0.9, 2.6);
        for (int i = 0; i < cl.n; ++i) {
          BoxObstacle obs; obs.x = cx(rng); obs.y = cy(rng);
          obs.sx = sz(rng); obs.sy = sz(rng); obs.sz = h(rng); obs.z = obs.sz / 2.0;
          if (std::abs(obs.x) > 7.5 || std::abs(obs.y) > 6.5) continue;
          if (tooClose(obs.x, obs.y)) continue;
          obstacles_.push_back(obs);
        }
      }
    } else {
      std::uniform_real_distribution<double> px(-7.2, 7.2), py(-6.2, 6.2), sz(0.4, 0.85), h(0.9, 2.8);
      for (int i = 0; i < 40; ++i) {
        BoxObstacle obs; obs.x = px(rng); obs.y = py(rng);
        obs.sx = sz(rng); obs.sy = sz(rng); obs.sz = h(rng); obs.z = obs.sz / 2.0;
        if (tooClose(obs.x, obs.y)) continue;
        obstacles_.push_back(obs);
      }
    }
    obstacles_.push_back(BoxObstacle{-1.2, -0.5, 1.6, 1.0, 2.8, 3.2});
    obstacles_.push_back(BoxObstacle{ 1.5,  1.2, 1.5, 2.4, 1.0, 3.0});
    obstacles_.push_back(BoxObstacle{ 2.5, -1.8, 1.3, 1.0, 2.6, 2.6});
  }

  // ---- ESDF (only built when use_esdf == true) ----
  void buildEsdf() {
    esdf_nx_ = static_cast<int>((x_max_ - x_min_) / esdf_res_) + 1;
    esdf_ny_ = static_cast<int>((y_max_ - y_min_) / esdf_res_) + 1;
    esdf_.assign(esdf_nx_ * esdf_ny_, 0.0);
    for (int ix = 0; ix < esdf_nx_; ++ix) {
      double x = x_min_ + ix * esdf_res_;
      for (int iy = 0; iy < esdf_ny_; ++iy) {
        double y = y_min_ + iy * esdf_res_;
        double best = 1e9;
        for (const auto & o : obstacles_)
          best = std::min(best, std::max({std::abs(x - o.x) - o.sx / 2.0,
                                          std::abs(y - o.y) - o.sy / 2.0, 0.0}));
        esdf_[iy * esdf_nx_ + ix] = best;
      }
    }
  }
  double esdfV(double x, double y) const {
    double fx = (x - x_min_) / esdf_res_, fy = (y - y_min_) / esdf_res_;
    int ix = std::clamp((int)std::floor(fx), 0, esdf_nx_ - 2);
    int iy = std::clamp((int)std::floor(fy), 0, esdf_ny_ - 2);
    double tx = std::clamp(fx - ix, 0.0, 1.0), ty = std::clamp(fy - iy, 0.0, 1.0);
    return (esdf_[iy * esdf_nx_ + ix] * (1 - tx) + esdf_[iy * esdf_nx_ + ix + 1] * tx) * (1 - ty) +
           (esdf_[(iy + 1) * esdf_nx_ + ix] * (1 - tx) + esdf_[(iy + 1) * esdf_nx_ + ix + 1] * tx) * ty;
  }
  void esdfGrad(double x, double y, double & gx, double & gy) const {
    double h = esdf_res_;
    gx = (esdfV(x + h, y) - esdfV(x - h, y)) / (2 * h);
    gy = (esdfV(x, y + h) - esdfV(x, y - h)) / (2 * h);
  }

  // ---- EGO analytic gradient (no field) ----
  double boxClear(double x, double y, const BoxObstacle & o) const {
    return std::max({std::abs(x - o.x) - o.sx / 2.0, std::abs(y - o.y) - o.sy / 2.0, 0.0});
  }
  void egoGrad(double x, double y, double & gx, double & gy) const {
    double best = 1e9; const BoxObstacle * bo = nullptr;
    for (const auto & o : obstacles_) { double c = boxClear(x, y, o); if (c < best) { best = c; bo = &o; } }
    if (!bo || best >= d_safe_) { gx = gy = 0; return; }
    double hx = bo->sx / 2.0, hy = bo->sy / 2.0;
    double cx = std::clamp(x, bo->x - hx, bo->x + hx);
    double cy = std::clamp(y, bo->y - hy, bo->y + hy);
    double vx = x - cx, vy = y - cy, n = std::hypot(vx, vy);
    if (n < 1e-6) {
      double dxf = hx - std::abs(x - bo->x), dyf = hy - std::abs(y - bo->y);
      if (dxf <= dyf) { vx = (x >= bo->x) ? 1 : -1; vy = 0; }
      else { vx = 0; vy = (y >= bo->y) ? 1 : -1; }
      n = 1.0;
    }
    vx /= n; vy /= n;
    double coef = -2.0 * lambda_c_ * (d_safe_ - best);
    gx = coef * vx; gy = coef * vy;
  }

  BackendResult optimize(bool use_esdf)
  {
    BackendResult r;
    auto t0 = std::chrono::steady_clock::now();
    if (use_esdf) buildEsdf();
    auto t1 = std::chrono::steady_clock::now();
    r.build_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    auto q = cps_;
    auto t2 = std::chrono::steady_clock::now();
    for (int iter = 0; iter < opt_iters_; ++iter) {
      std::vector<Vec3> grad(q.size(), Vec3{0, 0, 0});
      // Smoothness.
      for (size_t i = 1; i + 1 < q.size(); ++i) {
        Vec3 a{q[i + 1].x - 2 * q[i].x + q[i - 1].x, q[i + 1].y - 2 * q[i].y + q[i - 1].y, 0};
        grad[i - 1].x += lambda_s_ * 2 * a.x; grad[i - 1].y += lambda_s_ * 2 * a.y;
        grad[i].x -= lambda_s_ * 4 * a.x; grad[i].y -= lambda_s_ * 4 * a.y;
        grad[i + 1].x += lambda_s_ * 2 * a.x; grad[i + 1].y += lambda_s_ * 2 * a.y;
      }
      // Collision.
      for (size_t i = 0; i < q.size(); ++i) {
        double gx, gy;
        if (use_esdf) {
          double d = esdfV(q[i].x, q[i].y);
          if (d >= d_safe_) continue;
          esdfGrad(q[i].x, q[i].y, gx, gy);
          double coef = -2.0 * lambda_c_ * (d_safe_ - d);
          grad[i].x += coef * gx; grad[i].y += coef * gy;
        } else {
          egoGrad(q[i].x, q[i].y, gx, gy);
          grad[i].x += gx; grad[i].y += gy;
        }
      }
      for (size_t i = 2; i + 2 < q.size(); ++i) {
        q[i].x -= opt_step_ * grad[i].x; q[i].y -= opt_step_ * grad[i].y;
        q[i].x = std::clamp(q[i].x, x_min_ + 0.2, x_max_ - 0.2);
        q[i].y = std::clamp(q[i].y, y_min_ + 0.2, y_max_ - 0.2); q[i].z = z0_;
      }
    }
    auto t3 = std::chrono::steady_clock::now();
    r.opt_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();
    r.total_ms = r.build_ms + r.opt_ms;

    r.path = sample(q);
    r.smooth = compSmooth(r.path);
    r.curv  = maxCurv(r.path);
    r.len   = compLen(r.path);
    return r;
  }

  void report()
  {
    double speedup = (ego_result_.total_ms > 1e-6) ? esdf_result_.total_ms / ego_result_.total_ms : 0;
    RCLCPP_INFO(this->get_logger(), "========== EGO vs ESDF ==========");
    RCLCPP_INFO(this->get_logger(), "ESDF | build: %.1f | opt: %.1f | total: %.1f ms | smooth: %.4f | curv: %.4f | len: %.2f",
      esdf_result_.build_ms, esdf_result_.opt_ms, esdf_result_.total_ms,
      esdf_result_.smooth, esdf_result_.curv, esdf_result_.len);
    RCLCPP_INFO(this->get_logger(), "EGO  | build: %.1f | opt: %.1f | total: %.1f ms | smooth: %.4f | curv: %.4f | len: %.2f",
      ego_result_.build_ms, ego_result_.opt_ms, ego_result_.total_ms,
      ego_result_.smooth, ego_result_.curv, ego_result_.len);
    RCLCPP_INFO(this->get_logger(), ">>> EGO speed-up: %.2fx <<<", speedup);
    RCLCPP_INFO(this->get_logger(), "==================================");
  }

  // ---- B-spline + metrics ----
  void generateControlPoints() {
    cps_.clear(); if (kino_path_.size() < 2) return;
    cps_.push_back(kino_path_.front()); cps_.push_back(kino_path_.front());
    for (const auto & p : kino_path_) cps_.push_back(p);
    cps_.push_back(kino_path_.back()); cps_.push_back(kino_path_.back());
  }
  geometry_msgs::msg::Point splinePt(const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Point & p1,
                                     const geometry_msgs::msg::Point & p2, const geometry_msgs::msg::Point & p3, double u) const {
    double u2 = u * u, u3 = u2 * u;
    geometry_msgs::msg::Point p;
    p.x = ((1 - 3 * u + 3 * u2 - u3) * p0.x + (4 - 6 * u2 + 3 * u3) * p1.x +
           (1 + 3 * u + 3 * u2 - 3 * u3) * p2.x + u3 * p3.x) / 6.0;
    p.y = ((1 - 3 * u + 3 * u2 - u3) * p0.y + (4 - 6 * u2 + 3 * u3) * p1.y +
           (1 + 3 * u + 3 * u2 - 3 * u3) * p2.y + u3 * p3.y) / 6.0;
    p.z = z0_; return p;
  }
  std::vector<geometry_msgs::msg::Point> sample(const std::vector<geometry_msgs::msg::Point> & q) const {
    std::vector<geometry_msgs::msg::Point> path;
    if (q.size() < 4) return path;
    for (size_t i = 0; i + 3 < q.size(); ++i)
      for (int j = 0; j < bspline_seg_; ++j)
        path.push_back(splinePt(q[i], q[i + 1], q[i + 2], q[i + 3], (double)j / bspline_seg_));
    path.push_back(q.back()); return path;
  }
  double compSmooth(const std::vector<geometry_msgs::msg::Point> & p) const {
    if (p.size() < 3) return 0; double c = 0;
    for (size_t i = 1; i + 1 < p.size(); ++i) {
      double ax = p[i + 1].x - 2 * p[i].x + p[i - 1].x, ay = p[i + 1].y - 2 * p[i].y + p[i - 1].y;
      c += ax * ax + ay * ay;
    }
    return c;
  }
  double compLen(const std::vector<geometry_msgs::msg::Point> & p) const {
    double l = 0; for (size_t i = 1; i < p.size(); ++i) l += std::hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
    return l;
  }
  double maxCurv(const std::vector<geometry_msgs::msg::Point> & p) const {
    if (p.size() < 3) return 0; double mc = 0;
    for (size_t i = 1; i + 1 < p.size(); ++i) {
      double dx = p[i + 1].x - p[i - 1].x, dy = p[i + 1].y - p[i - 1].y;
      double ddx = p[i + 1].x - 2 * p[i].x + p[i - 1].x, ddy = p[i + 1].y - 2 * p[i].y + p[i - 1].y;
      mc = std::max(mc, std::abs(dx * ddy - dy * ddx) / (std::pow(dx * dx + dy * dy, 1.5) + 1e-6));
    }
    return mc;
  }

  // ============ front-end (abbreviated, same as ackermann_kino_astar_demo) ==========
  double adiff(double a, double b) const { double d = a - b; while (d > M_PI) d -= 2 * M_PI; while (d < -M_PI) d += 2 * M_PI; return d; }
  bool isObs(double x, double y) const {
    for (const auto & o : obstacles_)
      if (std::abs(x - o.x) <= o.sx / 2.0 + safety_ && std::abs(y - o.y) <= o.sy / 2.0 + safety_) return true;
    return false;
  }
  bool valid(const AckermannState & s) const { return s.x >= x_min_ && s.x < x_max_ && s.y >= y_min_ && s.y < y_max_ && !isObs(s.x, s.y) && std::abs(s.v) <= max_vel_ + 1e-6; }
  long long key(const AckermannState & s) const {
    int ix = static_cast<int>((s.x - x_min_) / pos_res_); int iy = static_cast<int>((s.y - y_min_) / pos_res_);
    if (ix < 0 || ix >= nx_ || iy < 0 || iy >= ny_) return -1;
    double th = std::fmod(s.theta, 2 * M_PI); if (th < 0) th += 2 * M_PI;
    int ith = static_cast<int>(th / theta_res_); if (ith < 0 || ith >= ntheta_) return -1;
    int iv = static_cast<int>(std::round((s.v + max_vel_) / vel_res_)); if (iv < 0 || iv >= nv_) return -1;
    long long kk = ix; kk = kk * ny_ + iy; kk = kk * ntheta_ + ith; kk = kk * nv_ + iv; return kk;
  }
  AckermannState prop(const AckermannState & s, double vc, double de, double dt) const {
    double v = std::clamp(vc, -max_vel_, max_vel_), d = std::clamp(de, -max_steer_, max_steer_);
    double dth = v * std::tan(d) / wheelbase_ * dt;
    AckermannState ns; ns.x = s.x + v * std::cos(s.theta + 0.5 * dth) * dt;
    ns.y = s.y + v * std::sin(s.theta + 0.5 * dth) * dt;
    ns.theta = std::fmod(s.theta + dth, 2 * M_PI); if (ns.theta < 0) ns.theta += 2 * M_PI; ns.v = v; return ns;
  }
  bool runKinoAStar() {
    if (!valid(start_state_)) return false;
    std::vector<KinoNode> nd; std::priority_queue<QueueItem> op;
    std::unordered_map<long long, double> bg; std::unordered_map<long long, int> bi; std::unordered_set<long long> cl;
    long long sk = key(start_state_);
    KinoNode sn; sn.s = start_state_; sn.time = 0; sn.g_score = 0; sn.f_score = std::hypot(start_state_.x - goal_x_, start_state_.y - goal_y_) / max_vel_; sn.parent = -1; sn.key = sk;
    nd.push_back(sn); bg[sk] = 0; bi[sk] = 0; op.push({sn.f_score, 0});
    std::vector<std::pair<double,double>> cs;
    for (double v : {0.5 * v_cruise_, v_cruise_}) for (double s : {-max_steer_, -0.5 * max_steer_, 0.0, 0.5 * max_steer_, max_steer_}) cs.push_back({v, s});
    cs.push_back({-0.4 * v_cruise_, 0.0});
    int fid = -1, exp = 0;
    while (!op.empty() && exp < max_expand_) {
      auto it = op.top(); op.pop(); int ci = it.node_id; const KinoNode cur = nd[ci];
      if (bi.find(cur.key) == bi.end() || bi[cur.key] != ci) continue; if (cl.count(cur.key)) continue; cl.insert(cur.key); exp++;
      if (std::hypot(cur.s.x - goal_x_, cur.s.y - goal_y_) < goal_tol_ && cur.time > 0.5 && std::abs(adiff(cur.s.theta, std::atan2(goal_y_ - cur.s.y, goal_x_ - cur.s.x))) < goal_theta_tol_) { fid = ci; break; }
      if (cur.time > max_time_) continue;
      for (const auto & c : cs) {
        bool ok = true; for (int i = 1; i <= primitive_check_ && ok; ++i) if (!valid(prop(cur.s, c.first, c.second, dt_ * i / primitive_check_))) ok = false;
        if (!ok) continue;
        AckermannState ns = prop(cur.s, c.first, c.second, dt_);
        long long nk = key(ns); if (nk < 0 || cl.count(nk)) continue;
        double ng = cur.g_score + dt_ + 0.02 * std::abs(c.second) * dt_;
        auto gi = bg.find(nk); if (gi == bg.end() || ng < gi->second) {
          KinoNode nn; nn.s = ns; nn.time = cur.time + dt_; nn.g_score = ng;
          nn.f_score = ng + std::hypot(ns.x - goal_x_, ns.y - goal_y_) / max_vel_ + 0.15 * std::abs(adiff(ns.theta, std::atan2(goal_y_ - ns.y, goal_x_ - ns.x)));
          nn.parent = ci; nn.key = nk;
          int id = static_cast<int>(nd.size()); nd.push_back(nn); bg[nk] = ng; bi[nk] = id; op.push({nn.f_score, id});
        }
      }
    }
    kino_path_.clear(); if (fid < 0) return false;
    std::vector<int> ids; int id = fid; while (id >= 0) { ids.push_back(id); id = nd[id].parent; }
    std::reverse(ids.begin(), ids.end());
    for (int nid : ids) { geometry_msgs::msg::Point p; p.x = nd[nid].s.x; p.y = nd[nid].s.y; p.z = z0_; kino_path_.push_back(p); }
    return true;
  }

  void timerCallback() {
    visualization_msgs::msg::MarkerArray arr; int id = 0;
    for (const auto & obs : obstacles_) { visualization_msgs::msg::Marker m;
      m.header.frame_id = "map"; m.header.stamp = now(); m.ns = "obs"; m.id = id++;
      m.type = visualization_msgs::msg::Marker::CUBE; m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = obs.x; m.pose.position.y = obs.y; m.pose.position.z = obs.z; m.pose.orientation.w = 1;
      m.scale.x = obs.sx; m.scale.y = obs.sy; m.scale.z = obs.sz;
      m.color.r = 0.8; m.color.g = 0.1; m.color.b = 0.1; m.color.a = 0.75; arr.markers.push_back(m);
    }
    auto mk = [&](const std::string & ns, int i, const std::vector<geometry_msgs::msg::Point> & pts,
                  double w, double r, double g, double b) {
      visualization_msgs::msg::Marker m; m.header.frame_id = "map"; m.header.stamp = now();
      m.ns = ns; m.id = i; m.type = visualization_msgs::msg::Marker::LINE_STRIP; m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = w; m.pose.orientation.w = 1; m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1; m.points = pts; arr.markers.push_back(m);
    };
    mk("kino", 2000, kino_path_, 0.05, 0.1, 0.9, 0.1);
    mk("esdf", 3000, esdf_result_.path, 0.055, 1.0, 0.55, 0.0);
    mk("ego",  3100, ego_result_.path, 0.055, 0.2, 0.6, 1.0);
    marker_pub_->publish(arr);
  }

  double x_min_, x_max_, y_min_, y_max_, z0_, pos_res_, theta_res_, vel_res_; int nx_, ny_, ntheta_, nv_;
  double wheelbase_, max_vel_, max_steer_, kappa_max_, v_cruise_, dt_; int primitive_check_;
  double goal_tol_, goal_theta_tol_, safety_, max_time_; int max_expand_;
  AckermannState start_state_; double goal_x_, goal_y_; std::string scenario_;
  int bspline_seg_, opt_iters_; double opt_step_, lambda_s_, lambda_c_, lambda_k_, d_safe_, esdf_res_;
  int esdf_nx_, esdf_ny_; std::vector<double> esdf_;
  std::vector<BoxObstacle> obstacles_;
  std::vector<geometry_msgs::msg::Point> kino_path_, cps_;
  BackendResult esdf_result_, ego_result_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<AckermannEgoDemoNode>()); rclcpp::shutdown(); return 0; }