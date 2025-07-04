/**
 * @file trajectory.h
 * @brief Trajectory class
 */

#ifndef MPL_TRAJECTORY_H
#define MPL_TRAJECTORY_H

#include <mpl_basis/lambda.h>
#include <mpl_basis/primitive.h>
#include <mpl_basis/data_type.h>

/**
 * @brief Command class
 *
 * State includes position, velocity, acceleration and jerk in \f$R^n\f$, where
 * the dimension \f$n\f$ can be either 2 or 3. Yaw and yaw_dot are also
 * contained.
 */
template <int Dim>
struct Command {
  Vecf<Dim> pos;      ///< position in \f$R^{Dim}\f$
  Vecf<Dim> vel;      ///< velocity in \f$R^{Dim}\f$
  Vecf<Dim> acc;      ///< acceleration in \f$R^{Dim}\f$
  Vecf<Dim> jrk;      ///< jerk in \f$R^{Dim}\f$
  decimal_t yaw;      ///< yaw
  decimal_t yaw_dot;  ///< yaw velocity
  decimal_t t;        /// Time \f$t\f$ wrt when evaluate
};

/// Command 2D
typedef Command<2> Command2D;

/// Command 3D
typedef Command<3> Command3D;

/**
 * @brief Trajectory class
 *
 * A trajectory is composed by multiple end-to-end connected primitives,
 * so-called piece-wise polynomials
 */
template <int Dim>
class Trajectory {
 public:
  /**
   * @brief Empty constructor
   */
  Trajectory() : total_t_(0) {}
  /**
   * @brief Construct from multiple primitives
   */
  Trajectory(const vec_E<Primitive<Dim>> &prs) : segs(prs) {
    taus.push_back(0);
    for (const auto &pr : prs) taus.push_back(pr.t() + taus.back());
    Ts = taus;
    total_t_ = taus.back();
  }

  /**
   * @brief Evaluate Waypoint at time \f$t\f$
   *
   * If t is out of scope, we set t to be the closer bound (0 or total_t_) and
   * return the evaluation; The failure case is when lambda is ill-posed such
   * that \f$t = \lambda(\tau)^{-1}\f$ has no solution, in which a null Waypoint
   * is returned
   */
  Waypoint<Dim> evaluate(decimal_t time) const {
    decimal_t tau = lambda_.getTau(time);
    if (tau < 0) tau = 0;
    if (tau > total_t_) tau = total_t_;

    for (size_t id = 0; id < segs.size(); id++) {
      if ((tau >= taus[id] && tau < taus[id + 1]) || id == segs.size() - 1) {
        tau -= taus[id];
        Waypoint<Dim> p(segs[id].control());
        for (int j = 0; j < Dim; j++) {
          const auto pr = segs[id].pr(j);
          p.pos(j) = pr.p(tau);
          p.vel(j) = pr.v(tau);
          p.acc(j) = pr.a(tau);
          p.jrk(j) = pr.j(tau);
          p.yaw = normalize_angle(segs[id].pr_yaw().p(tau));
        }
        return p;
      }
    }

    printf("cannot find tau according to time: %f\n", time);
    return Waypoint<Dim>();
  }

  /**
   * @brief Evaluate Command at \f$t\f$, return false if fails to evaluate
   *
   * If \f$t\f$ is out of scope, we set \f$t\f$ to be the closer bound (0 or
   * total_t_) and return the evaluation; The failure case is when lambda is
   * ill-posed such that \f$t = \lambda(\tau)^{-1}\f$ has no solution.
   */
  bool evaluate(decimal_t time, Command<Dim> &p) const {
    decimal_t tau = lambda_.getTau(time);
    if (tau < 0) tau = 0;
    if (tau > total_t_) tau = total_t_;

    decimal_t lambda = 1;
    decimal_t lambda_dot = 0;

    if (lambda_.exist()) {
      VirtualPoint vt = lambda_.evaluate(tau);
      lambda = vt.p;
      lambda_dot = vt.v;
    }

    for (size_t id = 0; id < segs.size(); id++) {
      if (tau >= taus[id] && tau <= taus[id + 1]) {
        tau -= taus[id];
        for (int j = 0; j < Dim; j++) {
          const auto pr = segs[id].pr(j);
          p.pos(j) = pr.p(tau);
          p.vel(j) = pr.v(tau) / lambda;
          p.acc(j) = pr.a(tau) / lambda / lambda -
                     p.vel(j) * lambda_dot / lambda / lambda / lambda;
          p.jrk(j) = pr.j(tau) / lambda / lambda -
                     3 / power(lambda, 3) * p.acc(j) * p.acc(j) * lambda_dot +
                     3 / power(lambda, 4) * p.vel(j) * lambda_dot * lambda_dot;
          p.yaw = normalize_angle(segs[id].pr_yaw().p(tau));
          p.yaw_dot = normalize_angle(segs[id].pr_yaw().v(tau));
          p.t = time;
        }
        return true;
      }
    }

    printf("cannot find tau according to time: %f\n", time);
    return false;
  }

  /*bool quad_evaluate(decimal_t time, Command<Dim> &p, Command<Dim> &quad_p) const {
    decimal_t tau = lambda_.getTau(time);
    if (tau < 0) tau = 0;
    if (tau > total_t_) tau = total_t_;

    decimal_t lambda = 1;
    decimal_t lambda_dot = 0;

    if (lambda_.exist()) {
      VirtualPoint vt = lambda_.evaluate(tau);
      lambda = vt.p;
      lambda_dot = vt.v;
    }

    for (size_t id = 0; id < segs.size(); id++) {
      if (tau >= taus[id] && tau <= taus[id + 1]) {
        tau -= taus[id];
        for (int j = 0; j < Dim; j++) {
          const auto pr = segs[id].pr(j);
          p.pos(j) = pr.p(tau);
          p.vel(j) = pr.v(tau) / lambda;
          p.acc(j) = pr.a(tau) / lambda / lambda -
                     p.vel(j) * lambda_dot / lambda / lambda / lambda;
          p.jrk(j) = pr.j(tau) / lambda / lambda -
                     3 / power(lambda, 3) * p.acc(j) * p.acc(j) * lambda_dot +
                     3 / power(lambda, 4) * p.vel(j) * lambda_dot * lambda_dot;
          p.yaw = normalize_angle(segs[id].pr_yaw().p(tau));
          p.yaw_dot = normalize_angle(segs[id].pr_yaw().v(tau));
          p.t = time;
        }
        decimal_t norm_w = sqrt(pow(p.acc(0),2)+pow(p.acc(1),2)+pow(p.acc(2),2));
        decimal_t len = 0.6;
        for (int j = 0; j < Dim; j++) {
            if (j == 2)
            Vec6f coeff = segs[id].get_quad_stateforz(p.pos(j), p.vel(j), p.acc(j), p.jrk(j), decimal_t(0), len, norm_w);
            else
            Vec6f coeff = segs[id].get_quad_state(p.pos(j), p.vel(j), p.acc(j), p.jrk(j), decimal_t(0), len, norm_w);
            quad_p.pos(j) = coeff(5);
            quad_p.vel(j) = coeff(4);
            quad_p.acc(j) = coeff(3);
            quad_p.jrk(j) = coeff(2);
        }
        quad_p.yaw = p.yaw;
        quad_p.yaw_dot = p.yaw_dot;
        quad_p.t = p.t;
        return true;
      }
    }

    printf("cannot find tau according to time: %f\n", time);
    return false;
  }*/

  bool quad_evaluate(decimal_t time, Command<Dim> &p, Command<Dim> &quad_p) const {
    decimal_t tau = lambda_.getTau(time);
    if (tau < 0) tau = 0;
    if (tau > total_t_) tau = total_t_;

    decimal_t lambda = 1;
    decimal_t lambda_dot = 0;

    if (lambda_.exist()) {
      VirtualPoint vt = lambda_.evaluate(tau);
      lambda = vt.p;
      lambda_dot = vt.v;
    }

    for (int id = 0; id < segs.size(); id++) {
      if (tau >= taus[id] && tau <= taus[id + 1]) {
        tau -= taus[id];
        for (int j = 0; j < Dim; j++) {
          const auto pr = segs[id].pr(j);
          p.pos(j) = pr.p(tau);
          p.vel(j) = pr.v(tau) / lambda;
          p.acc(j) = pr.a(tau) / lambda / lambda -
                     p.vel(j) * lambda_dot / lambda / lambda / lambda;
          p.jrk(j) = pr.j(tau) / lambda / lambda -
                     3 / power(lambda, 3) * p.acc(j) * p.acc(j) * lambda_dot +
                     3 / power(lambda, 4) * p.vel(j) * lambda_dot * lambda_dot;
          p.yaw = normalize_angle(segs[id].pr_yaw().p(tau));
          p.yaw_dot = normalize_angle(segs[id].pr_yaw().v(tau));
        //   p.yaw = 0;
        //   p.yaw_dot = 0;
          p.t = time;
        }
        decimal_t norm_w = sqrt(pow(p.acc(0),2)+pow(p.acc(1),2)+pow((p.acc(2)+9.8),2));
        decimal_t len = 0.6;
        for (int j = 0; j < Dim; j++) {
            Vec6f coeff;
            if (j == 2)
            coeff = get_quad_stateforz(p.pos(j), p.vel(j), p.acc(j), p.jrk(j), 0, len, norm_w);
            else
            coeff = get_quad_state(p.pos(j), p.vel(j), p.acc(j), p.jrk(j), 0, len, norm_w);

            quad_p.pos(j) = coeff(5);
            quad_p.vel(j) = coeff(4);
            quad_p.acc(j) = coeff(3);
            quad_p.jrk(j) = coeff(2);
        }
        quad_p.yaw = p.yaw;
        quad_p.yaw_dot = p.yaw_dot;
        quad_p.t = p.t;
        return true;
      }
    }

    printf("cannot find tau according to time: %f\n", time);
    return false;
  }

    //@note:计算无人机位置速度和加速度
  Vec6f get_quad_state(decimal_t pos, decimal_t vel, decimal_t acc, 
  decimal_t jrk, decimal_t snp, decimal_t len, decimal_t norm_w) const {
    Vec6f quad_state;
    double quad_pos = pos + acc*len/norm_w;
    double quad_vel = vel + len*(jrk/norm_w-(acc*acc*jrk/pow(norm_w,3)));
    double quad_acc = acc + len*(snp/norm_w-(3*acc*jrk*jrk+snp*acc*acc)/pow(norm_w,3)+3*pow(acc,3)*jrk*jrk/pow(norm_w,5));
    // printf("coeff value: %f %f %f \n",acc*acc*jrk/pow(norm_w,3), (3*acc*jrk*jrk+snp*acc*acc)/pow(norm_w,3), 3*pow(acc,3)*jrk*jrk/pow(norm_w,5));
    // printf("value of load: %f %f %f value of quad: %f %f %f \n", pos, vel, acc, quad_pos, quad_vel, quad_acc);
    quad_state << 0,0,0,quad_acc,quad_vel,quad_pos;
    return quad_state;
  }

  Vec6f get_quad_stateforz(decimal_t pos, decimal_t vel, decimal_t acc, 
    decimal_t jrk, decimal_t snp, decimal_t len, decimal_t norm_w) const {
    Vec6f quad_state;
    decimal_t accz = acc+9.8;
    double quad_pos = pos + accz*len/norm_w;
    double quad_vel = vel + len*(jrk/norm_w-(accz*accz*jrk/pow(norm_w,3)));
    double quad_acc = acc + len*(snp/norm_w-(3*accz*jrk*jrk+snp*accz*accz)/pow(norm_w,3)+3*pow(accz,3)*jrk*jrk/pow(norm_w,5));
    // printf("coeff value: %f %f %f \n",acc*acc*jrk/pow(norm_w,3), (3*acc*jrk*jrk+snp*acc*acc)/pow(norm_w,3), 3*pow(acc,3)*jrk*jrk/pow(norm_w,5));
    // printf("value of load: %f %f %f value of quad: %f %f %f \n", pos, vel, acc, quad_pos, quad_vel, quad_acc);
    quad_state << 0,0,0,quad_acc,quad_vel,quad_pos;
    return quad_state;
  }

  /**
   * @brief Scale according to ratio at start and end (velocity only)
   */
  bool scale(decimal_t ri, decimal_t rf) {
    std::vector<VirtualPoint> vs;
    VirtualPoint vi, vf;
    vi.p = 1.0 / ri;
    vi.v = 0;
    vi.t = 0;

    vf.p = 1.0 / rf;
    vf.v = 0;
    vf.t = taus.back();

    vs.push_back(vi);
    vs.push_back(vf);
    Lambda ls(vs);
    lambda_ = ls;
    std::vector<decimal_t> ts;
    for (const auto &tau : taus) ts.push_back(lambda_.getT(tau));
    Ts = ts;
    total_t_ = Ts.back();
    return true;
  }

  /**
   * @brief Scale down the whole trajectory according to mv
   */
  bool scale_down(decimal_t mv, decimal_t ri, decimal_t rf) {
    std::vector<VirtualPoint> vs;
    VirtualPoint vi, vf;
    vi.p = ri;
    vi.v = 0;
    vi.t = 0;

    vf.p = rf;
    vf.v = 0;
    vf.t = taus.back();

    vs.push_back(vi);
    for (int id = 0; id < (int)segs.size(); id++) {
      for (int i = 0; i < 3; i++) {
        if (segs[id].max_vel(i) > mv) {
          std::vector<decimal_t> ts = segs[id].pr(i).extrema_vel(segs[id].t());
          if (id != 0) ts.push_back(0);
          ts.push_back(segs[id].t());
          for (const auto &tv : ts) {
            Vec4f p = segs[id].pr(i).evaluate(tv);
            decimal_t v = p(1);
            decimal_t lambda_v = fabs(v) / mv;
            if (lambda_v <= 1) continue;

            VirtualPoint vt;
            vt.p = lambda_v;
            vt.v = 0;
            vt.t = tv + taus[id];
            vs.push_back(vt);
          }
        }
      }
    }

    vs.push_back(vf);

    std::sort(
        vs.begin(), vs.end(),
        [](const VirtualPoint &i, const VirtualPoint &j) { return i.t < j.t; });
    decimal_t max_l = 1;
    for (const auto &v : vs) {
      if (v.p > max_l) max_l = v.p;
    }

    if (max_l <= 1) return false;

    // printf("max_l: %f\n", max_l);
    for (int i = 1; i < (int)vs.size() - 1; i++) vs[i].p = max_l;
    std::vector<VirtualPoint> vs_s;
    vs_s.push_back(vs.front());
    for (const auto &v : vs)
      if (v.t > vs_s.back().t) vs_s.push_back(v);

    lambda_ = Lambda(vs_s);

    std::vector<decimal_t> ts;
    for (const auto &tau : taus) ts.push_back(lambda_.getT(tau));
    Ts = ts;
    total_t_ = Ts.back();
    return true;
  }

  /**
   * @brief Sample N+1 Command along trajectory using uniformed time
   */
  vec_E<Command<Dim>> sample(int N) const {
    vec_E<Command<Dim>> ps(N + 1);
    

    decimal_t dt = total_t_ / N;
    for (int i = 0; i <= N; i++) evaluate(i * dt, ps[i]);

    return ps;
  }

  vec_E<Command<Dim>> quad_sample(int N) const {
    vec_E<Command<Dim>> ps(N + 1);
    vec_E<Command<Dim>> temp_p(N + 1);

    decimal_t dt = total_t_ / N;
    for (int i = 0; i <= N; i++) quad_evaluate(i * dt, temp_p[i], ps[i]);

    return ps;
  }


  /**
   * @brief Return total efforts of Primitive for the given duration: \f$J(i) =
   * \int_0^T |p^{i}(t)|^2dt\f$
   * @param control Flag that indicates the order of derivative \f$i\f$
   *
   * Return J is the summation of efforts in all three dimensions.
   * `Control::VEL` or `Control::VELxYAW` corresponds to \f$i = 1\f$;
   * `Control::ACC` or `Control::ACCxYAW` corresponds to \f$i = 2\f$;
   * `Control::JRK` or `Control::JRKxYAW` corresponds to \f$i = 3\f$;
   * `Control::SNP` or `Control::SNPxYAW` corresponds to \f$i = 4\f$.
   */
  decimal_t J(const Control::Control &control) const {
    decimal_t j = 0;
    for (const auto &seg : segs) j += seg.J(control);
    return j;
  }

  /**
   * @brief Return total yaw efforts for the given duration: \f$J_{yaw} =
   * \int_0^T |\dot{\phi}(t)|^2dt\f$
   *
   * By default, the `Jyaw` is using `Control::VEL` to calculate.
   */
  decimal_t Jyaw() const {
    decimal_t j = 0;
    for (const auto &seg : segs) j += seg.Jyaw();
    return j;
  }

  /// Get time of each segment
  std::vector<decimal_t> getSegmentTimes() const {
    std::vector<decimal_t> dts;
    for (int i = 0; i < (int)Ts.size() - 1; i++)
      dts.push_back(Ts[i + 1] - Ts[i]);
    return dts;
  }

  /// Get intermediate Waypoint on the trajectory
  vec_E<Waypoint<Dim>> getWaypoints() const {
    vec_E<Waypoint<Dim>> ws;
    if (segs.empty()) return ws;
    decimal_t t = 0;
    for (const auto &seg : segs) {
      ws.push_back(seg.evaluate(0));
      ws.back().t = t;
      t += seg.t();
    }
    ws.push_back(segs.back().evaluate(segs.back().t()));
    ws.back().t = t;
    return ws;
  }

  /// Get Primitive array
  vec_E<Primitive<Dim>> getPrimitives() const { return segs; }

  /**
   * @brief Get the scaling factor Lambda
   */
  Lambda lambda() const { return lambda_; }

  /// Get the total duration of Trajectory
  decimal_t getTotalTime() const { return total_t_; }

  //@note:得到无人机轨迹
  Trajectory<Dim> get_quadtraj(){
      Trajectory<Dim> ws;
      float len_cable = 0.6;
      vec_E<Primitive<Dim>> new_segs;
      for (const auto &seg : segs){
        Primitive3D prss;
        for (int i = 0; i < 3; i++){
            // printf(" shengchangxiangliang: %f \n",seg.dir_cable(i));
            float temp = seg.prs_[i].c(5)+len_cable*seg.dir_cable(i);
            Vec6f temp_vec;
            temp_vec << 0, 0, 0, 0, 0, temp;
            prss.prs_[i] = Primitive1D(temp_vec);
        }
        prss.pr_yaw_ = seg.pr_yaw_;
        prss.t_ = seg.t_;
        prss.control_ = seg.control_;
        // seg.dir_cable = -seg.dir_cable;
        new_segs.push_back(prss);
      }
        ws.taus = taus;
        ws.Ts = Ts;
        ws.total_t_ = total_t_;
        ws.lambda_ = lambda_;
        ws.segs = new_segs;
        return ws;
  }

  void deleteE(Trajectory<Dim> &traj){
      traj.segs.pop_back();
  }

  /// Segments of primitives
  vec_E<Primitive<Dim>> segs;
  /// Time in virtual domain
  std::vector<decimal_t> taus;
  /// Time in actual domain
  std::vector<decimal_t> Ts;
  /// Total time of the trajectory
  decimal_t total_t_;
  /// Scaling object
  Lambda lambda_;
};

/// Trajectory in 2D
typedef Trajectory<2> Trajectory2D;

/// Trajectory in 3D
typedef Trajectory<3> Trajectory3D;

#endif
