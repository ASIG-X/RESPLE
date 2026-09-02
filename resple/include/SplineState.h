/*
This file is part of RESPLE: Recursive Spline Estimation for LiDAR-Based Odometry.
 
Copyright (c) 2026 Ziyu Cao

Developer: Ziyu Cao <ziyu.cao@liu.se>

For commercial use, please contact me at <ziyu.cao@liu.se> or Kailai Li at <kailai.li@liu.se>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file, 
which is included as part of this source code package.
*/

#pragma once

#include "utils/common_utils.h"
#include "utils/math_tools.h"
#include "estimate_msgs/msg/spline.hpp"
#include "estimate_msgs/msg/knot.hpp"

template <class MatT>
struct JacobianStruct {
    size_t start_idx;
    std::vector<MatT> d_val_d_knot;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef JacobianStruct<double> Jacobian;
typedef JacobianStruct<Eigen::Matrix<double, 4, 3>> Jacobian43;
typedef JacobianStruct<Eigen::Matrix3d> Jacobian33;

class SplineState
{

  public:

    SplineState() {};

    void init(int64_t dt_ns_, int num_knot_, int64_t start_t_ns_, int start_i_ = 0, 
              Eigen::Vector3d t0 = Eigen::Vector3d::Zero(), Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity())
    {
        if_first = true;
        dt_ns = dt_ns_;
        start_t_ns = start_t_ns_;
        num_knot = num_knot_;
        inv_dt = 1e9 / dt_ns;
        start_i = start_i_;
        pow_inv_dt[0] = 1.0;
        pow_inv_dt[1] = inv_dt;
        pow_inv_dt[2] = inv_dt * inv_dt;
        pow_inv_dt[3] = pow_inv_dt[2] * inv_dt;
        t_idle = {t0, t0, t0};
        ort_delta_idle = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
        q_idle = {q0, q0, q0};
    }

    void setTimeIntervalNs(int64_t t_ns)
    {
        dt_ns = t_ns;
        inv_dt = 1e9 / dt_ns;
        pow_inv_dt[0] = 1.0;
        pow_inv_dt[1] = inv_dt;
        pow_inv_dt[2] = inv_dt * inv_dt;
        pow_inv_dt[3] = pow_inv_dt[2] * inv_dt;
    }    

    Eigen::Matrix<double, 24, 1> getRCPs()
    {
        Eigen::Matrix<double, 24, 1> RCPs;
        for (int i = 0; i < 4; i++) {
            RCPs.block<3, 1>(i*6, 0) = t_knots[num_knot - 4 + i];
            RCPs.block<3, 1>(i*6+3, 0) = ort_delta[num_knot - 4 + i];
        }      
        return RCPs;
    }

    void updateRCPs(const Eigen::Matrix<double, 24, 1>& cp)
    {
        for (int i = 0; i < 4; i++) {
            t_knots[num_knot - 4 + i] = cp.block<3, 1>(i*6, 0);
            ort_delta[num_knot - 4 + i] = cp.block<3, 1>(i*6 + 3, 0);
            Eigen::Quaterniond q_del;
            Quater::exp(ort_delta[num_knot - 4 + i], q_del);
            if (int(num_knot) - 4 + i > 0) {
                q_knots[num_knot - 4 + i] = q_knots[num_knot - 5 + i] * q_del;
            } else if (int(num_knot) - 4 + i == 0) {
                q_knots[num_knot - 4 + i] = q_idle[2] * q_del;
            } else {
                assert(false && "num_knot - 4 + i  < 0");
            }
        }
    }    

    void addOneStateKnot(Eigen::Vector3d pos, Eigen::Vector3d ort_del)
    {
        t_knots.push_back(pos);
        ort_delta.push_back(ort_del);
        Eigen::Quaterniond ort;
        Eigen::Quaterniond q_del;
        Quater::exp(ort_del, q_del);
        if (num_knot > 0) {
            ort = q_knots.back() * q_del;
        } else {
            ort = q_idle[2] * q_del;
        }
        q_knots.push_back(ort);
        num_knot++;
    }    

    void setIdles(int idx, const Eigen::Vector3d& t, const Eigen::Vector3d& ort_del, const Eigen::Quaterniond& q_idle0)
    {
        t_idle[idx] = t;
        ort_delta_idle[idx] = ort_del;
        Eigen::Quaterniond q_del;
        Quater::exp(ort_del, q_del);        
        if (idx == 2) {
            assert(num_knot > 0);
            q_knots[0] = q_idle[2] * q_del;
        } else if (idx == 1) {
            q_idle[2] = q_idle[1] * q_del;
        } else if (idx == 0) {
            q_idle[0] = q_idle0;
            q_idle[1] = q_idle[0] * q_del;
        }
    }    

    void setOneStateKnot(int i, const Eigen::Vector3d& pos, const Eigen::Vector3d& ort_del)
    {
        t_knots[i] = pos;

        ort_delta[i] = ort_del;
        Eigen::Quaterniond q_del;
        Quater::exp(ort_del, q_del);
        if (i > 0) {
            q_knots[i] = q_knots[i - 1] * q_del;
        } else if (i == 0) {
            q_knots[i] = q_idle[2] * q_del;
        } else {
            assert(false && "i  < 0");
        }
    }    

    void updateKnots(SplineState* other)
    {
        int64_t num_knots_other = other->numKnots();
        Eigen::Quaterniond ref_ort_old;
        Eigen::Vector3d ref_trans_old;
        Eigen::Quaterniond ref_ort_new;
        Eigen::Vector3d ref_trans_new;
        if (num_knot <= num_knots_other) {
            q_idle = other->q_idle;
            ort_delta_idle = other->ort_delta_idle;
            t_idle = other->t_idle;
        }        
        for (int64_t i = 0; i < num_knots_other; i++) {
            if (i + other->start_i < num_knot) {
                setOneStateKnot(i + other->start_i, other->t_knots[i], other->ort_delta[i]);
            } else {
                addOneStateKnot(other->t_knots[i], other->ort_delta[i]);
            }
        }
    }    

    int64_t getKnotTimeNs(size_t i) const
    {
        return start_t_ns + i * dt_ns;
    }

    int64_t getKnotTimeIntervalNs() const
    {
        return dt_ns;
    }

    Eigen::Vector3d getKnotPos(size_t i) const
    {
        return t_knots[i];
    }

    Eigen::Quaterniond getKnotOrt(size_t i) const
    {
        return q_knots.at(i);
    }    

    Eigen::Vector3d getKnotOrtDel(size_t i) const
    {
        return ort_delta.at(i);
    }       

    int64_t maxTimeNs() const
    {
        if (num_knot == 1) {
           return start_t_ns;
        }
        return start_t_ns + (num_knot - 1) * dt_ns - 1;
    }

    int64_t minTimeNs() const
    {
        return start_t_ns + dt_ns * (!if_first ?  -1 : 0);
    }

    int64_t numKnots() const
    {
        return num_knot;
    }

    template <int Derivative = 0>
    Eigen::Vector3d itpPosition (int64_t time_ns, Jacobian* J = nullptr) const
    {
        return itpEuclidean<Eigen::Vector3d, Derivative>(time_ns, t_idle, t_knots, J);
    }

    void itpQuaternion(int64_t t_ns, Eigen::Quaterniond* q_out = nullptr,
                       Eigen::Vector3d* w_out = nullptr, Jacobian43* J_q = nullptr, Jacobian33* J_w = nullptr) const
    {
        double u;
        int64_t idx0;
        int idx_r;
        std::array<Eigen::Vector3d, 4> t_delta;
        prepareInterpolation(t_ns, ort_delta_idle, ort_delta, idx0, u, t_delta, idx_r);
        Eigen::Vector4d p;
        Eigen::Vector4d coeff;
        baseCoeffsWithTime<0>(p, u);
        coeff = cumulative_blending_matrix * p;

        Eigen::Quaterniond cp0;
        if (idx_r > 3) {
            cp0 = q_knots[idx0-1];
        } else if (idx_r == 3) {
            cp0 = q_idle[2];
        } else if (idx_r == 2) {
            cp0 = q_idle[1];
        } else if (idx_r == 1) {
            cp0 = q_idle[0];
        } else {
            assert(false);
        }

        Eigen::Vector3d t_delta_scale[4];
        Eigen::Quaterniond q_delta_scale[4];
        Eigen::Quaterniond q_itps[4];
        Eigen::Vector3d w_itps[4];
        Eigen::Vector4d dcoeff;
        if (J_q || J_w) {
            Eigen::Matrix<double, 4, 3> dexp_dt[4];
            t_delta_scale[0] = t_delta[0] * coeff[0];
            t_delta_scale[1] = t_delta[1] * coeff[1];
            t_delta_scale[2] = t_delta[2] * coeff[2];
            t_delta_scale[3] = t_delta[3] * coeff[3];
            Quater::dexp(t_delta_scale[0], q_delta_scale[0], dexp_dt[0]);
            Quater::dexp(t_delta_scale[1], q_delta_scale[1], dexp_dt[1]);
            Quater::dexp(t_delta_scale[2], q_delta_scale[2], dexp_dt[2]);
            Quater::dexp(t_delta_scale[3], q_delta_scale[3], dexp_dt[3]);
            int size_J = std::min(idx_r + 1, 4);
            Eigen::Quaterniond q_r_all[4];
            q_r_all[3] = Eigen::Quaterniond::Identity();
            for (int i = 2; i >= 0; i-- ) {
                q_r_all[i] = q_delta_scale[i+1] * q_r_all[i+1];
            }
            if (J_q) {
                q_itps[0] = cp0 * q_delta_scale[0];
                q_itps[1] = q_itps[0] * q_delta_scale[1];
                q_itps[2] = q_itps[1] * q_delta_scale[2];
                q_itps[3] = q_itps[2] * q_delta_scale[3];
                q_itps[3].normalize();
                *q_out = q_itps[3];
                Eigen::Matrix4d Q_l_all[4];
                Quater::Qleft(cp0, Q_l_all[0]);
                Quater::Qleft(q_itps[0], Q_l_all[1]);
                Quater::Qleft(q_itps[1], Q_l_all[2]);
                Quater::Qleft(q_itps[2], Q_l_all[3]);
                J_q->d_val_d_knot.resize(size_J);
                J_q->start_idx = idx0;
                for (int i = size_J - 1; i >= 0; i--) {
                    Eigen::Matrix4d Q_r_all;
                    Quater::Qright(q_r_all[i], Q_r_all);
                    J_q->d_val_d_knot[i].noalias() = coeff[i] * Q_r_all * Q_l_all[i] * dexp_dt[i];
                }
            }
            if (J_w) {
                baseCoeffsWithTime<1>(p, u);
                dcoeff = inv_dt * cumulative_blending_matrix * p;
                w_itps[0].setZero();
                w_itps[1] = 2 * dcoeff[1] * t_delta[1];
                w_itps[2] = q_delta_scale[2].inverse() * w_itps[1] + 2 * dcoeff[2] * t_delta[2];
                w_itps[3] = q_delta_scale[3].inverse() * w_itps[2] + 2 * dcoeff[3] * t_delta[3];
                *w_out = w_itps[3];
                Eigen::Matrix<double, 3, 4> drot_dq[2];
                Quater::drot(w_itps[1], q_delta_scale[2], drot_dq[0]);
                Quater::drot(w_itps[2], q_delta_scale[3], drot_dq[1]);
                J_w->d_val_d_knot.resize(size_J);
                J_w->start_idx = idx0;
                J_w->d_val_d_knot[0].setZero();           
                if (size_J > 1) {
                    J_w->d_val_d_knot[1] = 2 * dcoeff[1] * q_delta_scale[3].inverse().toRotationMatrix() * q_delta_scale[2].inverse().toRotationMatrix();
                    if (size_J > 2) {
                        Eigen::Matrix3d tmp = coeff[2] * drot_dq[0] * dexp_dt[2];
                        J_w->d_val_d_knot[2] = q_delta_scale[3].inverse().toRotationMatrix() * (tmp + 2 * dcoeff[2] * Eigen::Matrix3d::Identity());
                        if (size_J > 3) {
                            J_w->d_val_d_knot[3] = coeff[3] * drot_dq[1] * dexp_dt[3] + 2 * dcoeff[3] * Eigen::Matrix3d::Identity();
                        }
                    }

                }

            }
        } else {
            t_delta_scale[0] = t_delta[0] * coeff[0];
            t_delta_scale[1] = t_delta[1] * coeff[1];
            t_delta_scale[2] = t_delta[2] * coeff[2];
            t_delta_scale[3] = t_delta[3] * coeff[3];
            Quater::exp(t_delta_scale[0], q_delta_scale[0]);
            Quater::exp(t_delta_scale[1], q_delta_scale[1]);
            Quater::exp(t_delta_scale[2], q_delta_scale[2]);
            Quater::exp(t_delta_scale[3], q_delta_scale[3]);
            if (q_out) {
                q_itps[0] = cp0 * q_delta_scale[0];
                q_itps[1] = q_itps[0] * q_delta_scale[1];
                q_itps[2] = q_itps[1] * q_delta_scale[2];
                q_itps[3] = q_itps[2] * q_delta_scale[3];
                q_itps[3].normalize();
                *q_out = q_itps[3];
            }
            if (w_out) {
                baseCoeffsWithTime<1>(p, u);
                dcoeff = inv_dt * cumulative_blending_matrix * p;

                w_itps[0].setZero();
                w_itps[1] = 2 * dcoeff[1] * t_delta[1];
                w_itps[2] = q_delta_scale[2].inverse() * w_itps[1] + 2 * dcoeff[2] * t_delta[2];
                w_itps[3] = q_delta_scale[3].inverse() * w_itps[2] + 2 * dcoeff[3] * t_delta[3];
                *w_out = w_itps[3];
            }
        }
    }  

    void getSplineMsg(estimate_msgs::msg::Spline& spline_msg, const int start_i)
    {
        static int last_start_idx = 0;
        spline_msg.dt = dt_ns;
        int sidx = std::min(std::max((int)(num_knot - 5), 0), start_i);
        spline_msg.start_idx = std::min(sidx, last_start_idx+1);
        spline_msg.start_t = getKnotTimeNs(num_knot - 5);
        for (size_t i = spline_msg.start_idx; i < (size_t) num_knot; i++) {
            estimate_msgs::msg::Knot knot_msg;
            knot_msg.position.x = t_knots[i].x();
            knot_msg.position.y = t_knots[i].y();
            knot_msg.position.z = t_knots[i].z();
            knot_msg.orientation_del.x = ort_delta[i].x();
            knot_msg.orientation_del.y = ort_delta[i].y();
            knot_msg.orientation_del.z = ort_delta[i].z();
            spline_msg.knots.push_back(knot_msg);
        }
        for (int i = 0; i < 3; i++) {
            estimate_msgs::msg::Knot idle_msg;
            idle_msg.position.x = t_idle[i].x();
            idle_msg.position.y = t_idle[i].y();
            idle_msg.position.z = t_idle[i].z();
            idle_msg.orientation_del.x = ort_delta_idle[i].x();
            idle_msg.orientation_del.y = ort_delta_idle[i].y();
            idle_msg.orientation_del.z = ort_delta_idle[i].z();
            spline_msg.idles.push_back(idle_msg);
        }
        spline_msg.start_q.w = q_idle[0].w();
        spline_msg.start_q.x = q_idle[0].x();
        spline_msg.start_q.y = q_idle[0].y();
        spline_msg.start_q.z = q_idle[0].z();        
        if (num_knot < 5) {
            spline_msg.start_idx = 0;

        } 
        last_start_idx = std::max((int)spline_msg.start_idx, (int)(num_knot - 5));

    }    

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:

    bool if_first;
    static const Eigen::Matrix4d blending_matrix;
    static const Eigen::Matrix4d base_coefficients;
    static const Eigen::Matrix4d cumulative_blending_matrix;

    int64_t dt_ns;
    double inv_dt;
    std::array<double, 4> pow_inv_dt;
    int64_t num_knot;
    int64_t start_i;
    int64_t start_t_ns;

    std::array<Eigen::Vector3d, 3> t_idle;
    std::array<Eigen::Vector3d, 3> ort_delta_idle;
    Eigen::aligned_deque<Eigen::Quaterniond> q_idle;

    Eigen::aligned_deque<Eigen::Vector3d> t_knots;
    Eigen::aligned_deque<Eigen::Quaterniond> q_knots;
    Eigen::aligned_deque<Eigen::Vector3d> ort_delta;

    template <typename _KnotT, int Derivative = 0>
    _KnotT itpEuclidean(int64_t t_ns, const std::array<_KnotT, 3>& knots_idle,
                        const Eigen::aligned_deque<_KnotT>& knots, Jacobian* J = nullptr) const
    {
        double u;
        int64_t idx0;
        int idx_r;
        std::array<_KnotT,4> cps;
        prepareInterpolation(t_ns, knots_idle, knots, idx0, u, cps, idx_r);
        Eigen::Vector4d p, coeff;
        baseCoeffsWithTime<Derivative>(p, u);
        coeff = pow_inv_dt[Derivative] * (blending_matrix * p);
        _KnotT res_out = coeff[0] * cps.at(0) + coeff[1] * cps.at(1) + coeff[2] * cps.at(2) + coeff[3] * cps.at(3);
        if (J) {
            int size_J = std::min(idx_r + 1, 4);
            J->d_val_d_knot.resize(size_J);
            for (int i = 0; i < size_J; i++) {
                J->d_val_d_knot[i] = coeff[4 - size_J + i];
            }
            J->start_idx = idx0;
        }
        return res_out;
    }  

    template<typename _KnotT>
    void prepareInterpolation(int64_t t_ns, const std::array<_KnotT, 3>& knots_idle,
                              const Eigen::aligned_deque<_KnotT>& knots, int64_t& idx0, double& u,
                              std::array<_KnotT,4>& cps, int& idx_r) const
    {
        int64_t t_ns_rel = t_ns - start_t_ns;
        int idx_l = floor(double(t_ns_rel) / double(dt_ns));
        idx_r = idx_l + 1;
        idx0 = std::max(idx_l - 2, 0);

        for (int i = 0; i < 2 - idx_l; i++) {
            cps[i] = knots_idle[i + idx_l + 1];
        }
        int idx_window = std::max(0, 2 - idx_l);
        for (int i = 0; i < std::min(idx_l + 2, 4); i++) {
            cps[i + idx_window] = knots[idx0 + i];
        }
        u = (t_ns - start_t_ns - idx_l * dt_ns) / double(dt_ns);
    }

    template <int Derivative, class Derived>
    static void baseCoeffsWithTime(const Eigen::MatrixBase<Derived>& res_const, double t)
    {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 4);
        Eigen::MatrixBase<Derived>& res = const_cast<Eigen::MatrixBase<Derived>&>(res_const);
        res.setZero();
        res[Derivative] = base_coefficients(Derivative, Derivative);
        double ti = t;
        for (int j = Derivative + 1; j < 4; j++) {
            res[j] = base_coefficients(Derivative, j) * ti;
            ti = ti * t;
        }
    }

    template <bool _Cumulative = false>
    static Eigen::Matrix4d computeBlendingMatrix()
    {
        Eigen::Matrix4d m;
        m.setZero();
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                double sum = 0;
                for (int s = j; s < 4; ++s) {
                    sum += std::pow(-1.0, s - j) * binomialCoefficient(4, s - j) *
                    std::pow(4 - s - 1.0, 4 - 1.0 - i);
                }
                m(j, i) = binomialCoefficient(3, 3 - i) * sum;
            }
        }
        if (_Cumulative) {
            for (int i = 0; i < 4; i++) {
                for (int j = i + 1; j < 4; j++) {
                    m.row(i) += m.row(j);
                }
            }
        }
        uint64_t factorial = 1;
        for (int i = 2; i < 4; ++i) {
            factorial *= i;
        }
        return m / factorial;
    }

    constexpr static inline uint64_t binomialCoefficient(uint64_t n, uint64_t k)
    {
        if (k > n) return 0;
        uint64_t r = 1;
        for (uint64_t d = 1; d <= k; ++d) {
            r *= n--;
            r /= d;
        }
        return r;
    }

    static Eigen::Matrix4d computeBaseCoefficients()
    {
        Eigen::Matrix4d base_coeff;
        base_coeff.setZero();
        base_coeff.row(0).setOnes();
        int order = 3;
        for (int n = 1; n < 4; n++) {
            for (int i = 3 - order; i < 4; i++) {
                base_coeff(n, i) = (order - 3 + i) * base_coeff(n - 1, i);
            }
            order--;
        }
        return base_coeff;
    }
};

const Eigen::Matrix4d SplineState::base_coefficients = SplineState::computeBaseCoefficients();
const Eigen::Matrix4d SplineState::blending_matrix = SplineState::computeBlendingMatrix();
const Eigen::Matrix4d SplineState::cumulative_blending_matrix = SplineState::computeBlendingMatrix<true>();