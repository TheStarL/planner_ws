// ackermann_closed_loop_demo.cpp
//
// Task 15-④ Ackermann variant: full closed-loop "perception -> planning -> control".
//
//   perception : box-obstacle map (static, with optional dynamic movers).
//   planning   : Ackermann Kinodynamic A* front-end + EGO B-spline back-end.
//   control    : Pure Pursuit tracking of the time-parameterised trajectory.
//                (v_cmd = cruise speed, δ = atan(2·L·sin(α)/L_d))
//
// Published to /planner_core_demo/closed_loop_markers for RViz.
// Robot state to /closed_loop_demo/odom.
// Ackermann command to /ackermann_cmd (for future Gazebo integration).

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

struct Vec3 { double x; double y; double z; };
struct BoxObstacle { double x; double y; double z; double sx; double sy; double sz; };
struct AckermannState { double x, y, theta, v; };
struct KinoNode { AckermannState s; double time, g_score, f_score; int parent; long long key; };
struct QueueItem { double f_score; int node_id; bool operator<(const QueueItem & o) const { return f_score > o.f_score; } };
struct DesiredState { double x, y, theta, v; };

class AckermannClosedLoopDemoNode : public rclcpp::Node
{
public:
  AckermannClosedLoopDemoNode() : Node("ackermann_closed_loop_demo_node")
  {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/planner_core_demo/closed_loop_markers", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/closed_loop_demo/odom", 10);

    initParams();
    generateObstacles();
    if (dynamic_) assignObstacleVelocities();

    if (planTrajectory()) {
      robot_state_ = start_state_;
      planning_ok_ = true;
      RCLCPP_INFO(this->get_logger(), "Planning done. %zu ref points, %.2f s. Start loop.",
        ref_x_.size(), total_time_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed.");
    }

    timer_ = this->create_wall_timer(20ms, std::bind(&AckermannClosedLoopDemoNode::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "ackermann_closed_loop_demo_node started.");
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
    v_cruise_ = 1.5; dt_ = 0.4; primitive_check_ = 8;
    goal_tol_ = 0.65; goal_theta_tol_ = M_PI / 6.0; safety_ = 0.22;
    max_time_ = 25.0; max_expand_ = 200000;
    start_state_.x = -7.0; start_state_.y = -6.0;
    start_state_.theta = std::atan2(6.0, 7.0); start_state_.v = 0.0;
    goal_x_ = 7.0; goal_y_ = 6.0;
    nv_ = static_cast<int>(2.0 * max_vel_ / vel_res_) + 1;
    scenario_ = this->declare_parameter("scenario", std::string("dense"));

    bspline_seg_ = 12; opt_iters_ = 300; replan_iters_ = 120;
    opt_step_ = 0.06; lambda_s_ = 1.0; lambda_c_ = 2.2; d_safe_ = 0.55;
    cruise_speed_ = 1.2;   // m/s, tracking cruise

    // Pure Pursuit parameters.
    look_ahead_ = 1.8;     // m
    ctrl_dt_ = 0.02; elapsed_time_ = 0.0; total_time_ = 0.0;
    kp_ = 4.0; kd_ = 3.6;  // for optional PD speed control
    reached_goal_ = false; planning_ok_ = false; step_ = 0;
    err_sum_ = 0.0; err_max_ = 0.0; err_n_ = 0; exec_safe_ = true;
    dynamic_ = this->declare_parameter("dynamic", true);
    replan_period_ = this->declare_parameter("replan_period", 0.25);
    last_replan_ = 0.0; wall_clock_ = 0.0; replan_count_ = 0;
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
          if (tooClose(obs.x, obs.y)) continue; obstacles_.push_back(obs);
        }
      }
    } else {
      std::uniform_real_distribution<double> px(-7.2, 7.2), py(-6.2, 6.2), sz(0.4, 0.85), h(0.9, 2.8);
      for (int i = 0; i < 40; ++i) {
        BoxObstacle obs; obs.x = px(rng); obs.y = py(rng);
        obs.sx = sz(rng); obs.sy = sz(rng); obs.sz = h(rng); obs.z = obs.sz / 2.0;
        if (tooClose(obs.x, obs.y)) continue; obstacles_.push_back(obs);
      }
    }
    obstacles_.push_back(BoxObstacle{-1.2, -0.5, 1.6, 1.0, 2.8, 3.2});
    obstacles_.push_back(BoxObstacle{ 1.5,  1.2, 1.5, 2.4, 1.0, 3.0});
    obstacles_.push_back(BoxObstacle{ 2.5, -1.8, 1.3, 1.0, 2.6, 2.6});
  }

  void assignObstacleVelocities()
  {
    obs_vel_.assign(obstacles_.size(), std::pair<double,double>(0, 0));
    obstacles_.push_back(BoxObstacle{-2.0, 1.5, 1.0, 0.7, 0.7, 2.0}); obs_vel_.push_back({0.55, -0.45});
    obstacles_.push_back(BoxObstacle{ 2.6, -1.8, 1.0, 0.7, 0.7, 2.0}); obs_vel_.push_back({-0.5, 0.55});
  }
  void updateObstacles(double dt)
  {
    if (!dynamic_) return;
    for (size_t i = 0; i < obstacles_.size(); ++i) {
      auto & v = obs_vel_[i]; if (std::abs(v.first) < 1e-9 && std::abs(v.second) < 1e-9) continue;
      obstacles_[i].x += v.first * dt; obstacles_[i].y += v.second * dt;
      if (obstacles_[i].x < -4.5 || obstacles_[i].x > 4.5) v.first = -v.first;
      if (obstacles_[i].y < -3.8 || obstacles_[i].y > 3.8) v.second = -v.second;
    }
  }

  // --- planners ---
  bool planTrajectory() { if (!runKinoAStar()) return false; genCps(); optCps_ = optimize(cps_, opt_iters_); opt_path_ = sample(optCps_); if (opt_path_.size() < 2) return false; timeAlloc(); return true; }
  void recedingReplan() { if (optCps_.size() < 4) return; optCps_ = optimize(optCps_, replan_iters_); opt_path_ = sample(optCps_); timeAlloc(); double best = 1e9; size_t bi = 0;
    for (size_t i = 0; i < ref_x_.size(); ++i) { double d = std::hypot(ref_x_[i] - robot_state_.x, ref_y_[i] - robot_state_.y); if (d < best) { best = d; bi = i; } }
    elapsed_time_ = ref_t_[bi]; replan_count_++; }

  // EGO B-spline back-end (2D, simplified)
  std::vector<geometry_msgs::msg::Point> optimize(const std::vector<geometry_msgs::msg::Point> & seed, int iters)
  {
    auto q = seed;
    for (int iter = 0; iter < iters; ++iter) {
      std::vector<Vec3> grad(q.size(), Vec3{0, 0, 0});
      for (size_t i = 1; i + 1 < q.size(); ++i) {
        Vec3 a{q[i + 1].x - 2 * q[i].x + q[i - 1].x, q[i + 1].y - 2 * q[i].y + q[i - 1].y, 0};
        grad[i - 1].x += lambda_s_ * 2 * a.x; grad[i - 1].y += lambda_s_ * 2 * a.y;
        grad[i].x -= lambda_s_ * 4 * a.x; grad[i].y -= lambda_s_ * 4 * a.y;
        grad[i + 1].x += lambda_s_ * 2 * a.x; grad[i + 1].y += lambda_s_ * 2 * a.y;
      }
      for (size_t i = 0; i < q.size(); ++i) {
        double best = 1e9; const BoxObstacle * bo = nullptr;
        for (const auto & o : obstacles_) { double c = clearance(q[i].x, q[i].y, o); if (c < best) { best = c; bo = &o; } }
        if (!bo || best >= d_safe_) continue;
        double hx = bo->sx / 2.0, hy = bo->sy / 2.0;
        double cx = std::clamp(q[i].x, bo->x - hx, bo->x + hx);
        double cy = std::clamp(q[i].y, bo->y - hy, bo->y + hy);
        double vx = q[i].x - cx, vy = q[i].y - cy, n = std::hypot(vx, vy);
        if (n < 1e-6) { vx = (q[i].x >= bo->x) ? 1 : -1; vy = 0; n = 1.0; }
        vx /= n; vy /= n;
        double coef = -2.0 * lambda_c_ * (d_safe_ - best);
        grad[i].x += coef * vx; grad[i].y += coef * vy;
      }
      for (size_t i = 2; i + 2 < q.size(); ++i) {
        q[i].x -= opt_step_ * grad[i].x; q[i].y -= opt_step_ * grad[i].y;
        q[i].x = std::clamp(q[i].x, x_min_ + 0.2, x_max_ - 0.2);
        q[i].y = std::clamp(q[i].y, y_min_ + 0.2, y_max_ - 0.2); q[i].z = z0_;
      }
    }
    return q;
  }

  void timeAlloc()
  {
    ref_x_.clear(); ref_y_.clear(); ref_t_.clear();
    for (const auto & p : opt_path_) { ref_x_.push_back(p.x); ref_y_.push_back(p.y); }
    ref_t_.assign(ref_x_.size(), 0.0);
    for (size_t i = 1; i < ref_x_.size(); ++i)
      ref_t_[i] = ref_t_[i - 1] + std::hypot(ref_x_[i] - ref_x_[i - 1], ref_y_[i] - ref_y_[i - 1]) / cruise_speed_;
    total_time_ = ref_t_.empty() ? 0.0 : ref_t_.back();
  }

  // -- Pure Pursuit --
  DesiredState purePursuit(double t) const
  {
    DesiredState d{};
    const size_t n = ref_x_.size(); if (n == 0) { d.x = robot_state_.x; d.y = robot_state_.y; d.theta = robot_state_.theta; d.v = 0; return d; }
    if (t >= total_time_ - 0.1) { d.x = ref_x_.back(); d.y = ref_y_.back(); d.theta = robot_state_.theta; d.v = 0; return d; }
    // Look ahead: find the nearest reference point at least look_ahead_ m away.
    double cx = robot_state_.x, cy = robot_state_.y;
    size_t bi = 0; while (bi < n && ref_t_[bi] < t) ++bi; if (bi >= n) bi = n - 1;
    for (size_t i = bi; i < n; ++i) {
      if (std::hypot(ref_x_[i] - cx, ref_y_[i] - cy) >= look_ahead_) {
        double alpha = std::atan2(ref_y_[i] - cy, ref_x_[i] - cx) - robot_state_.theta;
        while (alpha > M_PI) alpha -= 2 * M_PI; while (alpha < -M_PI) alpha += 2 * M_PI;
        d.x = ref_x_[i]; d.y = ref_y_[i];
        d.v = cruise_speed_;
        d.theta = std::atan2(2.0 * wheelbase_ * std::sin(alpha) / look_ahead_, 1.0);
        d.theta = std::clamp(d.theta, -max_steer_, max_steer_);
        return d;
      }
    }
    // Fallback: target the last point at reduced speed.
    d.x = ref_x_.back(); d.y = ref_y_.back();
    double alpha = std::atan2(d.y - cy, d.x - cx) - robot_state_.theta;
    d.v = 0.3; d.theta = std::atan2(2.0 * wheelbase_ * std::sin(alpha) / look_ahead_, 1.0);
    d.theta = std::clamp(d.theta, -max_steer_, max_steer_);
    return d;
  }

  void controlStep()
  {
    DesiredState d = purePursuit(elapsed_time_);
    double v_cmd = d.v;
    double steer = d.theta;
    // Bicycle kinematics integration.
    double dtheta = v_cmd * std::tan(steer) / wheelbase_ * ctrl_dt_;
    robot_state_.x += v_cmd * std::cos(robot_state_.theta + 0.5 * dtheta) * ctrl_dt_;
    robot_state_.y += v_cmd * std::sin(robot_state_.theta + 0.5 * dtheta) * ctrl_dt_;
    robot_state_.theta = std::fmod(robot_state_.theta + dtheta, 2 * M_PI);
    if (robot_state_.theta < 0) robot_state_.theta += 2 * M_PI;
    robot_state_.v = v_cmd;

    error_ = std::hypot(robot_state_.x - d.x, robot_state_.y - d.y);
    err_sum_ += error_ * error_; err_n_++;
    err_max_ = std::max(err_max_, error_);
    if (isObstacle(robot_state_.x, robot_state_.y)) { exec_safe_ = false; collision_steps_++; }

    geometry_msgs::msg::Point ap; ap.x = robot_state_.x; ap.y = robot_state_.y; ap.z = z0_;
    actual_path_.push_back(ap); if (actual_path_.size() > 4000) actual_path_.erase(actual_path_.begin());

    current_desired_ = d;
    if (!reached_goal_ && std::hypot(robot_state_.x - goal_x_, robot_state_.y - goal_y_) < 0.3 && std::abs(robot_state_.v) < 0.5) {
      reached_goal_ = true;
      double rms = (err_n_ > 0) ? std::sqrt(err_sum_ / err_n_) : 0.0;
      RCLCPP_INFO(this->get_logger(), "===== CLOSED-LOOP REPORT =====");
      RCLCPP_INFO(this->get_logger(), "goal: t=%.2f s | replans: %d | RMS err: %.4f m | max err: %.4f m | safe: %s",
        wall_clock_, replan_count_, rms, err_max_, exec_safe_ ? "true" : "false");
    }
  }

  // ================ front-end (Kino A*, same as other ackermann nodes) ==========
  double clearance(double x, double y, const BoxObstacle & o) const { return std::max({std::abs(x - o.x) - o.sx / 2.0, std::abs(y - o.y) - o.sy / 2.0, 0.0}); }
  double adiff(double a, double b) const { double d = a - b; while (d > M_PI) d -= 2 * M_PI; while (d < -M_PI) d += 2 * M_PI; return d; }
  bool isObstacle(double x, double y) const { for (const auto & o : obstacles_) if (std::abs(x - o.x) <= o.sx / 2.0 + safety_ && std::abs(y - o.y) <= o.sy / 2.0 + safety_) return true; return false; }
  bool valid(const AckermannState & s) const { return s.x >= x_min_ && s.x < x_max_ && s.y >= y_min_ && s.y < y_max_ && !isObstacle(s.x, s.y) && std::abs(s.v) <= max_vel_ + 1e-6; }
  long long key(const AckermannState & s) const { int ix = (int)((s.x - x_min_) / pos_res_); int iy = (int)((s.y - y_min_) / pos_res_); if (ix < 0 || ix >= nx_ || iy < 0 || iy >= ny_) return -1;
    double th = std::fmod(s.theta, 2 * M_PI); if (th < 0) th += 2 * M_PI; int ith = (int)(th / theta_res_); if (ith < 0 || ith >= ntheta_) return -1;
    int iv = (int)std::round((s.v + max_vel_) / vel_res_); if (iv < 0 || iv >= nv_) return -1;
    long long kk = ix; kk = kk * ny_ + iy; kk = kk * ntheta_ + ith; kk = kk * nv_ + iv; return kk; }
  AckermannState prop(const AckermannState & s, double vc, double de, double dt) const { double v = std::clamp(vc, -max_vel_, max_vel_), d = std::clamp(de, -max_steer_, max_steer_);
    double dth = v * std::tan(d) / wheelbase_ * dt; AckermannState ns; ns.x = s.x + v * std::cos(s.theta + 0.5 * dth) * dt;
    ns.y = s.y + v * std::sin(s.theta + 0.5 * dth) * dt; ns.theta = std::fmod(s.theta + dth, 2 * M_PI); if (ns.theta < 0) ns.theta += 2 * M_PI; ns.v = v; return ns; }
  bool runKinoAStar() { if (!valid(start_state_)) return false; std::vector<KinoNode> nd; std::priority_queue<QueueItem> op;
    std::unordered_map<long long, double> bg; std::unordered_map<long long, int> bi; std::unordered_set<long long> cl;
    long long sk = key(start_state_); KinoNode sn; sn.s = start_state_; sn.time = 0; sn.g_score = 0;
    sn.f_score = std::hypot(start_state_.x - goal_x_, start_state_.y - goal_y_) / max_vel_;
    sn.parent = -1; sn.key = sk; nd.push_back(sn); bg[sk] = 0; bi[sk] = 0; op.push({sn.f_score, 0});
    std::vector<std::pair<double,double>> cs;
    for (double v : {0.5 * v_cruise_, v_cruise_}) for (double s : {-max_steer_, -0.5 * max_steer_, 0.0, 0.5 * max_steer_, max_steer_}) cs.push_back({v, s});
    cs.push_back({-0.4 * v_cruise_, 0.0}); int fid = -1, exp = 0;
    while (!op.empty() && exp < max_expand_) { auto it = op.top(); op.pop(); int ci = it.node_id; const KinoNode cur = nd[ci];
      if (bi.find(cur.key) == bi.end() || bi[cur.key] != ci) continue; if (cl.count(cur.key)) continue; cl.insert(cur.key); exp++;
      double dg = std::atan2(goal_y_ - cur.s.y, goal_x_ - cur.s.x);
      if (std::hypot(cur.s.x - goal_x_, cur.s.y - goal_y_) < goal_tol_ && cur.time > 0.5 && std::abs(adiff(cur.s.theta, dg)) < goal_theta_tol_) { fid = ci; break; }
      if (cur.time > max_time_) continue;
      for (const auto & c : cs) { bool ok = true; for (int i = 1; i <= primitive_check_ && ok; ++i) if (!valid(prop(cur.s, c.first, c.second, dt_ * i / primitive_check_))) ok = false; if (!ok) continue;
        AckermannState ns = prop(cur.s, c.first, c.second, dt_); long long nk = key(ns); if (nk < 0 || cl.count(nk)) continue;
        double ng = cur.g_score + dt_ + 0.02 * std::abs(c.second) * dt_; auto gi = bg.find(nk); if (gi == bg.end() || ng < gi->second) {
          KinoNode nn; nn.s = ns; nn.time = cur.time + dt_; nn.g_score = ng;
          nn.f_score = ng + std::hypot(ns.x - goal_x_, ns.y - goal_y_) / max_vel_ + 0.15 * std::abs(adiff(ns.theta, dg));
          nn.parent = ci; nn.key = nk; int id = static_cast<int>(nd.size()); nd.push_back(nn); bg[nk] = ng; bi[nk] = id; op.push({nn.f_score, id}); } } }
    kino_path_.clear(); if (fid < 0) return false; std::vector<int> ids; int id = fid;
    while (id >= 0) { ids.push_back(id); id = nd[id].parent; } std::reverse(ids.begin(), ids.end());
    for (int nid : ids) { geometry_msgs::msg::Point p; p.x = nd[nid].s.x; p.y = nd[nid].s.y; p.z = z0_; kino_path_.push_back(p); } return true; }
  void genCps() { cps_.clear(); if (kino_path_.size() < 2) return; cps_.push_back(kino_path_.front()); cps_.push_back(kino_path_.front());
    for (const auto & p : kino_path_) cps_.push_back(p); cps_.push_back(kino_path_.back()); cps_.push_back(kino_path_.back()); }
  geometry_msgs::msg::Point splinePt(const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, const geometry_msgs::msg::Point & p3, double u) const {
    double u2 = u * u, u3 = u2 * u; geometry_msgs::msg::Point p;
    p.x = ((1 - 3 * u + 3 * u2 - u3) * p0.x + (4 - 6 * u2 + 3 * u3) * p1.x + (1 + 3 * u + 3 * u2 - 3 * u3) * p2.x + u3 * p3.x) / 6.0;
    p.y = ((1 - 3 * u + 3 * u2 - u3) * p0.y + (4 - 6 * u2 + 3 * u3) * p1.y + (1 + 3 * u + 3 * u2 - 3 * u3) * p2.y + u3 * p3.y) / 6.0; p.z = z0_; return p; }
  std::vector<geometry_msgs::msg::Point> sample(const std::vector<geometry_msgs::msg::Point> & q) const { std::vector<geometry_msgs::msg::Point> p; if (q.size() < 4) return p;
    for (size_t i = 0; i + 3 < q.size(); ++i) for (int j = 0; j < bspline_seg_; ++j) p.push_back(splinePt(q[i], q[i + 1], q[i + 2], q[i + 3], (double)j / bspline_seg_)); p.push_back(q.back()); return p; }

  // ---- markers ----
  void timerCallback() {
    if (planning_ok_ && !reached_goal_) { updateObstacles(ctrl_dt_);
      if (dynamic_ && wall_clock_ - last_replan_ >= replan_period_) { recedingReplan(); last_replan_ = wall_clock_; }
      controlStep(); elapsed_time_ += ctrl_dt_; wall_clock_ += ctrl_dt_;
    }
    publishMarkers();
    if (++step_ % 50 == 0 && planning_ok_) RCLCPP_INFO(this->get_logger(), "t:%.2f pos:[%.2f,%.2f] err:%.3f replans:%d", wall_clock_, robot_state_.x, robot_state_.y, error_, replan_count_);
  }
  void publishMarkers() {
    visualization_msgs::msg::MarkerArray arr; int id = 0;
    for (size_t k = 0; k < obstacles_.size(); ++k) { const auto & o = obstacles_[k]; bool is_dyn = (k < obs_vel_.size() && (std::abs(obs_vel_[k].first) > 1e-9 || std::abs(obs_vel_[k].second) > 1e-9));
      visualization_msgs::msg::Marker m; m.header.frame_id = "map"; m.header.stamp = now(); m.ns = "obs"; m.id = id++; m.type = visualization_msgs::msg::Marker::CUBE; m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = o.x; m.pose.position.y = o.y; m.pose.position.z = o.z; m.pose.orientation.w = 1;
      m.scale.x = o.sx; m.scale.y = o.sy; m.scale.z = o.sz; m.color.r = is_dyn ? 1.0 : 0.8; m.color.g = is_dyn ? 0.55 : 0.1; m.color.b = 0.1; m.color.a = is_dyn ? 0.85 : 0.7; arr.markers.push_back(m);
    }
    auto mk = [&](const std::string & ns, int i, const std::vector<geometry_msgs::msg::Point> & pts, double w, double r, double g, double b) {
      visualization_msgs::msg::Marker m; m.header.frame_id = "map"; m.header.stamp = now(); m.ns = ns; m.id = i; m.type = visualization_msgs::msg::Marker::LINE_STRIP; m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = w; m.pose.orientation.w = 1; m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1; m.points = pts; arr.markers.push_back(m); };
    std::vector<geometry_msgs::msg::Point> ref; for (size_t i = 0; i < ref_x_.size(); ++i) { geometry_msgs::msg::Point p; p.x = ref_x_[i]; p.y = ref_y_[i]; p.z = z0_; ref.push_back(p); }
    mk("kino", 100, kino_path_, 0.05, 0.1, 0.85, 0.1); mk("ref", 101, ref, 0.09, 1.0, 0.85, 0.05); mk("actual", 102, actual_path_, 0.07, 0.0, 0.9, 1.0);
    arr.markers.push_back(sphere("sg", 200, start_state_.x, start_state_.y, 0.35, 0, 1, 0));
    arr.markers.push_back(sphere("sg", 201, goal_x_, goal_y_, 0.35, 0, 0.2, 1));
    arr.markers.push_back(sphere("robot", 300, robot_state_.x, robot_state_.y, 0.32, 1, 0.55, 0));
    arr.markers.push_back(sphere("desired", 301, current_desired_.x, current_desired_.y, 0.20, 1, 0, 1));
    marker_pub_->publish(arr);
  }
  visualization_msgs::msg::Marker sphere(const std::string & ns, int i, double x, double y, double s, double r, double g, double b) {
    visualization_msgs::msg::Marker m; m.header.frame_id = "map"; m.header.stamp = now(); m.ns = ns; m.id = i; m.type = visualization_msgs::msg::Marker::SPHERE; m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z0_; m.pose.orientation.w = 1; m.scale.x = m.scale.y = m.scale.z = s; m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1; return m;
  }

  double x_min_, x_max_, y_min_, y_max_, z0_, pos_res_, theta_res_, vel_res_; int nx_, ny_, ntheta_, nv_;
  double wheelbase_, max_vel_, max_steer_, v_cruise_, dt_; int primitive_check_;
  double goal_tol_, goal_theta_tol_, safety_, max_time_; int max_expand_;
  AckermannState start_state_; double goal_x_, goal_y_; std::string scenario_;
  int bspline_seg_, opt_iters_, replan_iters_; double opt_step_, lambda_s_, lambda_c_, d_safe_;
  double cruise_speed_, look_ahead_, ctrl_dt_, elapsed_time_, total_time_;
  double kp_, kd_; bool reached_goal_, planning_ok_; int step_;
  double err_sum_, err_max_; int err_n_; bool exec_safe_; double error_;
  DesiredState current_desired_;
  bool dynamic_; double replan_period_, last_replan_, wall_clock_; int replan_count_; int collision_steps_;
  std::vector<std::pair<double,double>> obs_vel_;
  AckermannState robot_state_;
  std::vector<BoxObstacle> obstacles_;
  std::vector<geometry_msgs::msg::Point> kino_path_, cps_, optCps_, opt_path_, actual_path_;
  std::vector<double> ref_x_, ref_y_, ref_t_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<AckermannClosedLoopDemoNode>()); rclcpp::shutdown(); return 0; }