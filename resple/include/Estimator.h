/*
This file is part of RESPLE: Recursive Spline Estimation for LiDAR-Based Odometry.
 
Copyright (c) 2026 Ziyu Cao

Developer: Ziyu Cao <ziyu.cao@liu.se>

For commercial use, please contact me at <ziyu.cao@liu.se> or Kailai Li at <kailai.li@liu.se>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file, 
which is included as part of this source code package.
*/

#pragma once

#include "SplineState.h"
#include "Association.h"

template<int XSIZE>
class Estimator
{
  public:
    static const int CP_SIZE = 24;
    static const int BA_OFFSET = 24;
    static const int BG_OFFSET = 27;  
    int n_iter = 1;

    Estimator() {};

    void setState(int64_t dt_ns, int64_t start_t_ns, const Eigen::Vector3d& t0, const Eigen::Quaterniond& q0, 
        const Eigen::Matrix<double, XSIZE, XSIZE>& Q, const Eigen::Matrix<double, XSIZE, XSIZE>& P)
    {
        spl.init(dt_ns, 0, start_t_ns, 0, t0, q0);
        for (int i = 0; i < 4; i++) {
            spl.addOneStateKnot(t0, Eigen::Vector3d::Zero());
        }        
        cov_sys = Q;
        cov_rcp = P;
        a_mat = Eigen::Matrix<double, XSIZE, XSIZE>::Zero();
        Eigen::Matrix<double, 6, 6> matblock = Eigen::Matrix<double, 6, 6>::Zero();
        matblock.topLeftCorner<3, 3>().setIdentity();
        matblock.bottomRightCorner<3, 3>().setIdentity();

        a_mat.block(0, 6, 6, 6) = matblock;
        a_mat.block(6, 12, 6, 6) = matblock;
        a_mat.block(12, 18, 6, 6) = matblock;
        a_mat.block(18, 0, 3, 3) = - Eigen::Matrix3d::Identity();
        a_mat.block(18, 12, 3, 3) = 2 * Eigen::Matrix3d::Identity();
        a_mat.block(21, 9, 3, 3) = Eigen::Matrix3d::Identity();   
        if constexpr (XSIZE == 30) {
            a_mat.block(BA_OFFSET, BA_OFFSET, 3, 3) = Eigen::Matrix3d::Identity();
            a_mat.block(BG_OFFSET, BG_OFFSET, 3, 3) = Eigen::Matrix3d::Identity();    
        }
    }

    Eigen::Matrix<double, XSIZE, 1> getState()
    {
        Eigen::Matrix<double, CP_SIZE, 1> cps_win = spl.getRCPs();
        Eigen::Matrix<double, XSIZE, 1> state;
        if constexpr (XSIZE == 24) {
            state = cps_win;
        } else {
            state << cps_win, ba, bg;
        }
        return state;
    }  

    void updateIEKFLiDAR(Eigen::aligned_deque<PointData>& pt_meas, KD_TREE<pcl::PointXYZINormal>* ikdtree, const double pt_thresh, const double cov_thresh)
    {
        const Eigen::Matrix<double, XSIZE, XSIZE> cov_prop = cov_rcp;
        Eigen::Matrix<double, XSIZE, 1> rcp_prop = getState();
        bool converged = true;
        int num_tot_eff = 0;
        int t = 0;
        for (int i = 0; i < max_iter; i++) {
            Eigen::Matrix<double, XSIZE, 1> rcpi = getState();
            if (converged) {
                num_tot_eff = 0;
                Association::findCorresp(num_tot_eff, &spl, ikdtree, pt_meas);
            }
            if (num_tot_eff > 0) {
                updateLiDAR(pt_meas, num_tot_eff, rcp_prop, cov_prop, pt_thresh, cov_thresh);
            } else {
                break;
            }
            converged = true;
            Eigen::Matrix<double, XSIZE, 1> state_af = getState();
            if ((state_af - rcpi).norm() > eps) {
                converged = false;
            } else {
                t++;
            }
            if(!t && i == max_iter - 2) {
                converged = true;
            }
            if ((t > n_iter) || (i == max_iter - 1)) {
                cov_rcp = ( Eigen::MatrixXd::Identity(XSIZE, XSIZE) - KH) * cov_prop;
                cov_rcp = 0.5*(cov_rcp + cov_rcp.transpose());
                break;
            }  
        }
    }

    void updateIEKFLiDARInertial(Eigen::aligned_deque<PointData>& pt_meas, KD_TREE<pcl::PointXYZINormal>* ikdtree, const double pt_thresh,
        Eigen::aligned_deque<ImuData>& imu_meas, const Eigen::Vector3d& g, const Eigen::Vector3d& cov_acc, const Eigen::Vector3d& cov_gyro, const double cov_thresh)
    {
        const Eigen::Matrix<double, XSIZE, XSIZE> cov_prop = cov_rcp;
        Eigen::Matrix<double, XSIZE, 1> rcp_prop = getState();
        bool converged = true;
        int num_tot_eff = 0;
        int t = 0;
        for (int i = 0; i < max_iter; i++) {
            Eigen::Matrix<double, XSIZE, 1> rcpi = getState();
            if (converged) {
                num_tot_eff = 0;
                Association::findCorresp(num_tot_eff, &spl, ikdtree, pt_meas);
            }
            if (num_tot_eff > 0 && imu_meas.empty()) {
                updateLiDAR(pt_meas, num_tot_eff, rcp_prop, cov_prop, pt_thresh, cov_thresh);
            } else if (num_tot_eff > 0) {
                updateLiDARInertial(pt_meas, imu_meas, num_tot_eff, rcp_prop, cov_prop, pt_thresh, cov_thresh, g, cov_acc, cov_gyro);
            } else {
                break;
            }
            converged = true;
            Eigen::Matrix<double, XSIZE, 1> state_af = getState();
            if ((state_af - rcpi).norm() > eps) {
                converged = false;
            } else {
                t++;
            }
            if(!t && i == max_iter - 2) {
                converged = true;
            }
            if ((t > n_iter) || (i == max_iter - 1)) {
                cov_rcp = ( Eigen::MatrixXd::Identity(XSIZE, XSIZE) - KH) * cov_prop;
                cov_rcp = 0.5*(cov_rcp + cov_rcp.transpose());
                break;
            }              
        }
    }

    void propRCP(int64_t t)
    {
        if (spl.maxTimeNs() >= t) {
            cov_rcp += cov_sys;
        } else {
            while (spl.maxTimeNs() < t) {
                Eigen::Matrix<double, 24, 1> cps_win = spl.getRCPs();
                Eigen::Matrix<double, 6, 1> cp_prop_pos = 2*cps_win.block<6, 1>(12, 0) - cps_win.block<6, 1>(0, 0);
                Eigen::Vector3d delta = cps_win.segment<3>(9);
                spl.addOneStateKnot(cp_prop_pos.head<3>(), delta);
                cov_rcp = a_mat * cov_rcp * a_mat.transpose() + cov_sys;
            }
        }
    }    

    SplineState* getSpline() {
        return &spl;
    }     

  private:
    SplineState spl;
    Eigen::Matrix<double, XSIZE, XSIZE> cov_rcp;  
    Eigen::Matrix<double, XSIZE, XSIZE> cov_sys; 
    Eigen::Matrix<double, XSIZE, XSIZE> a_mat;   
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();     
    Eigen::Matrix<double, XSIZE, XSIZE> KH;
    int max_iter = 5;
    double eps = 0.1;

    void prepIMU(ImuData& imu_data, const Eigen::Vector3d& g)
    {
        Eigen::Quaterniond q_itp;
        Eigen::Vector3d rot_vel;
        Jacobian43 J_ortdel;
        Jacobian J_line_acc;
        Jacobian33 J_gyro;
        spl.itpQuaternion(imu_data.time_ns, &q_itp, &rot_vel, &J_ortdel, &J_gyro);
        Eigen::Vector3d a_w_no_g = spl.itpPosition<2>(imu_data.time_ns, &J_line_acc);
        Eigen::Vector3d a_w = a_w_no_g + g;
        Eigen::Matrix3d RT = q_itp.inverse().toRotationMatrix();   
        Eigen::Matrix<double, 3, 4> drot;
        Quater::drot(a_w, q_itp, drot);
        Eigen::Matrix<double, 6, XSIZE> Hi = Eigen::Matrix<double, 6, XSIZE>::Zero();
        int recur_st_id = spl.numKnots() - 4;
        for (int i = 0; i < (int) J_line_acc.d_val_d_knot.size(); i++) {
            int j = J_line_acc.start_idx + i - recur_st_id;
            if (j >= 0) {
                Hi.block(0, j*6, 3, 3) = RT * J_line_acc.d_val_d_knot[i];
                Hi.block(0, j*6 + 3, 3, 3) = drot * J_ortdel.d_val_d_knot[i];
                Hi.block(3, j*6 + 3, 3, 3) = J_gyro.d_val_d_knot[i];                
            }
        }       
        Hi.block(0, BA_OFFSET, 3, 3) = Eigen::Matrix3d::Identity();
        Hi.block(3, BG_OFFSET, 3, 3) = Eigen::Matrix3d::Identity();            
        imu_data.imu_itp.head<3>() = RT * a_w + ba;
        imu_data.imu_itp.tail<3>() = rot_vel + bg;
        imu_data.H = Hi.template leftCols<24>();
    }    

    void prepLiDAR(PointData& pt_data) const    
    {
        if (pt_data.if_valid) { 
            Eigen::Matrix<double, 1, XSIZE> Hi = Eigen::Matrix<double, 1, XSIZE>::Zero();
            Eigen::Quaterniond q_itp;
            Jacobian43 J_ortdel;
            Jacobian J_pos;
            spl.itpQuaternion(pt_data.time_ns, &q_itp, nullptr, &J_ortdel);
            Eigen::Vector3d p_itp = spl.itpPosition(pt_data.time_ns, &J_pos);
            Eigen::Matrix3d R_IL = pt_data.q_bl.toRotationMatrix();
            Eigen::Vector3d pt_w = q_itp * (R_IL * pt_data.pt_b + pt_data.t_bl) + p_itp;
            pt_data.zp = pt_data.normvec.dot(pt_w) + pt_data.dist;

            Eigen::Matrix<double, 3, 4> drot;
            Quater::drotInv((R_IL * pt_data.pt_b + pt_data.t_bl), q_itp, drot);
            Eigen::Matrix<double, 1, 4> tmp = pt_data.normvec.transpose() * drot;
            int RCP_st_id = spl.numKnots() - 4;
            for (int i = 0; i < (int) J_pos.d_val_d_knot.size(); i++) {
                int j = (int) J_pos.start_idx + i - RCP_st_id;
                if (j >= 0) {
                    Hi.block(0, j*6, 1, 3) = pt_data.normvec.transpose() * J_pos.d_val_d_knot[i];
                    Hi.block(0, j*6 + 3, 1, 3) = tmp * J_ortdel.d_val_d_knot[i];
                }
            }  
            pt_data.H = Hi.template leftCols<24>();
        }
    } 

    void updateState(const Eigen::Matrix<double, XSIZE, 1>& xupd)
    {
        Eigen::Matrix<double, CP_SIZE, 1> cp_win = xupd.segment(0, CP_SIZE);
        spl.updateRCPs(cp_win);
        if constexpr (XSIZE == 30) {
            ba = xupd.segment(BA_OFFSET, 3);
            bg = xupd.segment(BG_OFFSET, 3);   
        }
    }        

    bool updateLiDAR(Eigen::aligned_deque<PointData>& pt_meas, int num_valid, const Eigen::Matrix<double, XSIZE, 1>& x_prop, 
        const Eigen::Matrix<double, XSIZE, XSIZE>& P_prop, const double pt_thresh, const double cov_thresh)
    {
        Eigen::Matrix<double, Eigen::Dynamic, XSIZE> H(num_valid, XSIZE);
        Eigen::Matrix<double, Eigen::Dynamic, 1> innv(num_valid, 1);
        Eigen::Matrix<double, Eigen::Dynamic, 1> mat_cov_inv(num_valid, 1);
        H.setZero();    
        innv.setZero();
        mat_cov_inv.setConstant(1/0.01);
        size_t num_pt = pt_meas.size();    
        #pragma omp parallel for num_threads(NUM_OF_THREAD) schedule(dynamic)
        for (size_t i = 0; i < num_pt; i++) {
            PointData& pt_data = pt_meas[i];
            prepLiDAR(pt_data);
        }        
        int idx_offset = 0;
        for(size_t i = 0; i < num_pt; i++) {
            const PointData& pt_data = pt_meas[i];
            if (pt_data.if_valid) {
                Eigen::Matrix<double, 1, XSIZE> Hi = Eigen::Matrix<double, 1, XSIZE>::Zero();
                Hi.template leftCols<24>() = pt_data.H;
                double lid_cov = Hi*cov_rcp*Hi.transpose() + pt_data.var_pt;
                if (abs(pt_data.zp) < pt_thresh || lid_cov < pt_data.var_pt*cov_thresh) {
                    innv(idx_offset) = - pt_data.zp;
                    H.row(idx_offset) = Hi;
                    
                } 
                mat_cov_inv(idx_offset) = 1/pt_data.var_pt;
                idx_offset++;
            }
        }        
        update(innv, mat_cov_inv, H, x_prop, P_prop);
        return true;
    }           

    void updateLiDARInertial(Eigen::aligned_deque<PointData>& pt_meas, Eigen::aligned_deque<ImuData>& imu_meas, int num_valid, const Eigen::Matrix<double, XSIZE, 1>& x_prop, 
        const Eigen::Matrix<double, XSIZE, XSIZE>& P_prop, const double pt_thresh, const double cov_thresh, const Eigen::Vector3d& g, const Eigen::Vector3d& cov_acc, const Eigen::Vector3d& cov_gyro)
    {
        Eigen::Matrix<double, 6, 1> cov_imu_inv =  Eigen::Matrix<double, 6, 1>(1/cov_acc[0], 1/cov_acc[1], 1/cov_acc[2], 1/cov_gyro[0], 1/cov_gyro[1], 1/cov_gyro[2]);
        #pragma omp parallel for num_threads(NUM_OF_THREAD) schedule(dynamic)
        for (size_t i = 0; i < pt_meas.size(); i++) {
            PointData& pt_data = pt_meas[i];
            prepLiDAR(pt_data); 
        }
        #pragma omp parallel for num_threads(NUM_OF_THREAD) 
        for (size_t i = 0; i < imu_meas.size(); i++) {
            prepIMU(imu_meas[i], g);
        }                
        int dim_meas = 6*imu_meas.size() + num_valid;
        Eigen::Matrix<double, Eigen::Dynamic, XSIZE> H(dim_meas, XSIZE);
        Eigen::Matrix<double, Eigen::Dynamic, 1> innv(dim_meas, 1);
        Eigen::Matrix<double, Eigen::Dynamic, 1> mat_cov_inv(dim_meas, 1);
        H.setZero();    
        innv.setZero();
        mat_cov_inv.setZero();
        int idx_offset = 0;
        size_t id_imu = 0;
        size_t id_pt = 0;
        for (size_t j = 0; j < imu_meas.size() + pt_meas.size(); j++) {
            if ((id_pt < pt_meas.size() && id_imu < imu_meas.size() && pt_meas[id_pt].time_ns < imu_meas[id_imu].time_ns) ||
                (id_pt < pt_meas.size() && id_imu >= imu_meas.size())) {
                    PointData& pt_data = pt_meas[id_pt];
                    if (pt_data.if_valid) {
                        Eigen::Matrix<double, 24, 24> cov = cov_rcp.template topLeftCorner<24, 24>();
                        double lid_cov = pt_data.H*cov*pt_data.H.transpose() + pt_data.var_pt;
                        if (abs(pt_data.zp) < pt_thresh || lid_cov < pt_data.var_pt*cov_thresh) {
                            innv(idx_offset) = - pt_data.zp;
                            H.block(idx_offset, 0, 1, 24) = pt_data.H;
                        } 
                        mat_cov_inv(idx_offset) = 1/pt_data.var_pt;
                        idx_offset++;
                    }
                    id_pt++;
            } else if ((id_pt < pt_meas.size() && id_imu < imu_meas.size() && pt_meas[id_pt].time_ns >= imu_meas[id_imu].time_ns) ||
                        (id_pt >= pt_meas.size() && id_imu < imu_meas.size())) {
                    const ImuData& imu_data = imu_meas[id_imu];
                    Eigen::Matrix<double, 6, 1> imu_itp = imu_data.imu_itp;
                    Eigen::Matrix<double, 6, XSIZE> Hi = Eigen::Matrix<double, 6, XSIZE>::Zero();
                    Hi.template leftCols<24>() = imu_data.H;
                    Hi.block(0, BA_OFFSET, 3, 3) = Eigen::Matrix3d::Identity();
                    Hi.block(3, BG_OFFSET, 3, 3) = Eigen::Matrix3d::Identity();   

                    Eigen::Matrix<double, 6, 1> imu;
                    imu.head<3>() = imu_data.accel;
                    imu.tail<3>() = imu_data.gyro;
                    for (int i = 0; i < 3; i++) {
                        if (abs(imu(i) - imu_itp(i)) > 10.0) {
                            imu(i) = 0;
                            imu_itp(i) = 0;
                            Hi.row(i).setZero();
                        } 
                        if (abs(imu(i+3) - imu_itp(i+3)) > 5.0) {
                            imu(i+3) = 0;
                            imu_itp(i+3) = 0;
                            Hi.row(i+3).setZero();
                        }                     
                    }
                    innv.segment<6>(idx_offset) = imu - imu_itp;
                    H.block(idx_offset, 0, 6, XSIZE) = Hi;
                    mat_cov_inv.segment<6>(idx_offset) = cov_imu_inv;  
                    idx_offset += 6;
                    id_imu++;
            }
        }        
        update(innv, mat_cov_inv, H, x_prop, P_prop);        
    }

    template <int RSIZE>
    void update(const Eigen::Matrix<double, RSIZE, 1>& innov, const Eigen::Matrix<double, RSIZE, 1>& R_inv, const Eigen::Matrix<double, RSIZE, XSIZE>& H, 
        const Eigen::Matrix<double, XSIZE, 1>& x_prop, const Eigen::Matrix<double, XSIZE, XSIZE>& cov_prop)
    {
        int num_pts = innov.rows();
        Eigen::Matrix<double, XSIZE, 1> RCPs_post;
        Eigen::MatrixXd I_X = Eigen::MatrixXd::Identity(XSIZE, XSIZE); 
        if (num_pts > XSIZE) {
            Eigen::Matrix<double, XSIZE, XSIZE> cov_rcp_inv = cov_prop.llt().solve(I_X);
            Eigen::Matrix<double, XSIZE, RSIZE> HT_R_inv;
            HT_R_inv.noalias() = (H.transpose().array().rowwise() * R_inv.transpose().array()).matrix();
            Eigen::Matrix<double, XSIZE, XSIZE> HT_R_inv_H;
            HT_R_inv_H.noalias() = HT_R_inv * H;

            Eigen::Matrix<double, XSIZE, XSIZE> S = HT_R_inv_H;
            S.noalias() += cov_rcp_inv;
            Eigen::Matrix<double, XSIZE, XSIZE> S_inv = S.llt().solve(I_X);
            Eigen::Matrix<double, XSIZE, RSIZE> K;
            K.noalias() = S_inv * HT_R_inv;

            KH.noalias() = S_inv * HT_R_inv_H;     
            Eigen::Matrix<double, XSIZE, 1> delta_cur = (getState() - x_prop);
            Eigen::Matrix<double, XSIZE, 1> deltax = KH * delta_cur + K * innov - delta_cur;      
            RCPs_post.noalias() = getState() + deltax;      
        } else {
            Eigen::Matrix<double, RSIZE, RSIZE> R = R_inv.cwiseInverse().asDiagonal();
            Eigen::Matrix<double, RSIZE, RSIZE> S;
            S.noalias() = H * cov_prop * H.transpose() + R;
            Eigen::Matrix<double, XSIZE, RSIZE> K;
            K.noalias() = cov_prop * H.transpose() * S.inverse();
            KH.noalias() = K * H;
            Eigen::Matrix<double, XSIZE, 1> delta_cur = (getState() - x_prop);
            Eigen::Matrix<double, XSIZE, 1> deltax = KH * delta_cur + K * innov - delta_cur;
            RCPs_post.noalias() = getState() + deltax;
        }
        updateState(RCPs_post);     
    }    
};
