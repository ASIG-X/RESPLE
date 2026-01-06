#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <thread>
#include <iostream>
#include <queue>
#include <string>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/int64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/service.hpp>
#include <std_srvs/srv/empty.hpp>
#include "livox_ros_driver/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_interfaces/msg/custom_msg.hpp"
#include "estimate_msgs/msg/calib.hpp"
#include "estimate_msgs/msg/estimate.hpp"
#include "mocap4r2_msgs/msg/markers.hpp"
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include "SplineState.h"

template<typename PointType>
class MappingBase
{
  public:

    std::mutex mtx;    
    LidarConfig lidar;
    MappingBase(rclcpp::Node::SharedPtr &nh, const LidarConfig& lidar_config) : lidar(lidar_config)
    {
        pub_global_map = nh->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", 2);
        ds_filter_each_scan.setLeafSize(0.2, 0.2, 0.2);
        pc_last.reset(new typename pcl::PointCloud<PointType>());
        pc_last_ds.reset(new typename pcl::PointCloud<PointType>());
        pc.reset(new typename pcl::PointCloud<PointType>());   
    }

    void processScan(SplineState* spl, const int64_t spl_window_st_ns)
    {
        int64_t t_end_ns = 0;
        rclcpp::Rate rate(20);
        while (!pc_L_buff.empty()) {
            t_end_ns = pc_L_buff.front().header.stamp + int64_t (pc_L_buff.front().points.back().intensity * float(1e6));
            mtx.lock();
            if (t_end_ns < spl->minTimeNs()) {
                pc_L_buff.pop_front();
                mtx.unlock(); 
            } else if (t_end_ns <= spl->maxTimeNs()) {
                transformCloud(pc_L_buff.front(), spl, pc);
                pc_L_buff.pop_front();
                mtx.unlock();                
                publishMap(pc, pub_global_map);
            } else {
                mtx.unlock(); 
                rate.sleep();
                break;
            }
        }
    }

    void publishMap(const typename pcl::PointCloud<PointType>::Ptr& pcs,
                         const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) const
    {
        sensor_msgs::msg::PointCloud2 msgs;
        pcl::toROSMsg(*pcs, msgs);
        msgs.header.frame_id = odom_id;
        publisher->publish(msgs);
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_map;

    PointType transformPoint(int64_t time_ns, const SplineState* spl, const PointType& pt_in) const
    {     
        Eigen::Quaterniond q_itp;
        Eigen::Vector3d t_itp;
        spl->itpQuaternion(time_ns, &q_itp);
        t_itp = spl->itpPosition(time_ns);
        Eigen::Vector3d p_body(pt_in.x, pt_in.y, pt_in.z);
        Eigen::Vector3d p_imu(lidar.q_bl * p_body + lidar.t_bl);
        Eigen::Vector3d p_global(q_itp * p_imu + t_itp);
        PointType point_world;
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = pt_in.intensity;
        point_world.curvature = pt_in.curvature;
        return point_world;
    }    

    void transformCloud(const typename pcl::PointCloud<PointType>& pc_in, SplineState* spl,
                       typename pcl::PointCloud<PointType>::Ptr pc_out) const
    {
        int64_t time_begin = rclcpp::Time(pc_in.header.stamp).nanoseconds();
        pc->clear();
        pc_out->points.resize(pc_in.size());
        for (size_t i = 0; i < pc_in.size(); i++) {
            const PointType& pt = pc_in.points[i];
            int64_t t_ns = int64_t(pt.intensity * float(1e6)) + time_begin;
            if (t_ns >= spl->minTimeNs() && t_ns <= spl->maxTimeNs()) {
                pc_out->points[i] = transformPoint(t_ns, spl, pt);
            }
        }
    }    

  protected:
    Eigen::aligned_deque<typename pcl::PointCloud<PointType>> pc_L_buff;
    typename pcl::PointCloud<PointType>::Ptr pc_last;
    typename pcl::PointCloud<PointType>::Ptr pc_last_ds;
    pcl::VoxelGrid<pcl::PointXYZINormal> ds_filter_each_scan;    
    const std::string frame_id = "body";
    const std::string odom_id = "world";
    typename pcl::PointCloud<PointType>::Ptr pc;

};


class OusterBuff : public MappingBase<pcl::PointXYZINormal>
{
  public:
  OusterBuff(rclcpp::Node::SharedPtr &nh, const LidarConfig& lidar_config) : MappingBase<pcl::PointXYZINormal>(nh, lidar_config)
    {
        pc_subscription_ouster = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->lidar.topic, 100, std::bind(&OusterBuff::ousterLidarCallback, this, std::placeholders::_1));         
        double lidar_time_offset = CommonUtils::readParam<double>(nh, "lidar_time_offset", 0.0);
        time_offset = 1e9*lidar_time_offset;        
    }

    void ousterLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr ouster_msg_in)
    {
        this->pc_last->clear();
        pcl::PointCloud<ouster_ros::Point>::Ptr pc_last_ouster(new pcl::PointCloud<ouster_ros::Point>());
        pcl::fromROSMsg(*ouster_msg_in, *pc_last_ouster);
        size_t plsize = pc_last_ouster->size();
        if (plsize == 0) return;
        this->pc_last->reserve(plsize);
        pcl::PointXYZINormal pt;
        for (uint i = 1; i < plsize; i++) {
            pt.x = pc_last_ouster->points[i].x;
            pt.y = pc_last_ouster->points[i].y;
            pt.z = pc_last_ouster->points[i].z;
            pt.intensity = float (pc_last_ouster->points[i].t) / float (1e6); // unit: ms
            pt.curvature = 0.1 * pc_last_ouster->points[i].intensity;

            if (pt.intensity >= 0) {
                this->pc_last->points.push_back(pt);
            }
        }
        this->pc_last->header.frame_id = this->frame_id;
        this->pc_last->header.stamp = rclcpp::Time(ouster_msg_in->header.stamp).nanoseconds() - time_offset;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*this->pc_last, *this->pc_last, indices);
        if (this->pc_last->points.empty()) return;
        ds_filter_each_scan.setInputCloud(pc_last);
        this->pc_last_ds->clear();
        ds_filter_each_scan.filter(*this->pc_last_ds);
        pc_last_ds->header.frame_id = this->frame_id;
        pc_last_ds->header.stamp = rclcpp::Time(ouster_msg_in->header.stamp).nanoseconds() - time_offset;
        mtx.lock();
        this->pc_L_buff.push_back(*pc_last_ds);
        mtx.unlock();
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription_ouster;
    int64_t time_offset = 0;
};

class Mid70AviaBuff : public MappingBase<pcl::PointXYZINormal>
{
  public:
  Mid70AviaBuff(rclcpp::Node::SharedPtr &nh, const LidarConfig& lidar_config) : MappingBase<pcl::PointXYZINormal>(nh, lidar_config)
    {
        pc_subscription_livox = nh->create_subscription<livox_ros_driver::msg::CustomMsg>(
            this->lidar.topic, 100, std::bind(&Mid70AviaBuff::livoxLidarCallback, this, std::placeholders::_1));
    }

    void livoxLidarCallback(const livox_ros_driver::msg::CustomMsg::SharedPtr livox_msg_in)
    {
        this->pc_last->clear();
        int plsize = livox_msg_in->point_num;
        if (plsize == 0) return;
        this->pc_last->reserve(plsize);
        pcl::PointXYZINormal pt;
        for (int i = 1; i < plsize; i++) {
            if ((livox_msg_in->points[i].tag & 0x30) == 0x10 || (livox_msg_in->points[i].tag & 0x30) == 0x00) {
                pt.x = livox_msg_in->points[i].x;
                pt.y = livox_msg_in->points[i].y;
                pt.z = livox_msg_in->points[i].z;
                pt.intensity = float (livox_msg_in->points[i].offset_time) / float (1e6); // unit: ms         
                pt.curvature = livox_msg_in->points[i].reflectivity;
                pc_last->points.push_back(pt);
            }
        }
        this->pc_last->header.frame_id = this->frame_id;
        this->pc_last->header.stamp = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*this->pc_last, *this->pc_last, indices);
        if (this->pc_last->points.empty()) return;
        ds_filter_each_scan.setInputCloud(pc_last);
        this->pc_last_ds->clear();
        ds_filter_each_scan.filter(*this->pc_last_ds);
        pc_last_ds->header.frame_id = this->frame_id;
        pc_last_ds->header.stamp = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        mtx.lock();
        this->pc_L_buff.push_back(*pc_last_ds);
        mtx.unlock();
    }

  private:
    rclcpp::Subscription<livox_ros_driver::msg::CustomMsg>::SharedPtr pc_subscription_livox;
};

class HAP360Buff : public MappingBase<pcl::PointXYZINormal>
{
public:
    HAP360Buff(rclcpp::Node::SharedPtr &nh, const LidarConfig& lidar_config) : MappingBase<pcl::PointXYZINormal>(nh, lidar_config)
    {
        pc_subscription_livox = nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            this->lidar.topic, 100, std::bind(&HAP360Buff::livoxLidarCallback, this, std::placeholders::_1));
    }

    void livoxLidarCallback(livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg_in)
    {
        this->pc_last->clear();
        int plsize = livox_msg_in->point_num;
        if (plsize == 0) return;
        this->pc_last->reserve(plsize);
        pcl::PointXYZINormal pt;
        for (int i = 1; i < plsize; i++) {
            if ((livox_msg_in->points[i].tag & 0x30) == 0x10 || (livox_msg_in->points[i].tag & 0x30) == 0x00) {
                pt.x = livox_msg_in->points[i].x;
                pt.y = livox_msg_in->points[i].y;
                pt.z = livox_msg_in->points[i].z;
                pt.intensity = float (livox_msg_in->points[i].offset_time) / float (1e6); // unit: ms         
                pt.curvature = livox_msg_in->points[i].reflectivity;
                pc_last->points.push_back(pt);
            }
        }
        this->pc_last->header.frame_id = this->frame_id;
        this->pc_last->header.stamp = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*this->pc_last, *this->pc_last, indices);
        if (this->pc_last->points.empty()) return;
        ds_filter_each_scan.setInputCloud(pc_last);
        this->pc_last_ds->clear();
        ds_filter_each_scan.filter(*this->pc_last_ds);
        pc_last_ds->header.frame_id = this->frame_id;
        pc_last_ds->header.stamp = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        mtx.lock();
        this->pc_L_buff.push_back(*pc_last_ds);
        mtx.unlock();
    }

  private:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr pc_subscription_livox;
};

class AviaRespleBuff : public MappingBase<pcl::PointXYZINormal>
{
public:
    AviaRespleBuff(rclcpp::Node::SharedPtr &nh, const LidarConfig& lidar_config) : MappingBase<pcl::PointXYZINormal>(nh, lidar_config)
    {
        pc_subscription_livox = nh->create_subscription<livox_interfaces::msg::CustomMsg>(
            this->lidar.topic, 100, std::bind(&AviaRespleBuff::livoxLidarCallback, this, std::placeholders::_1));
    }

    void livoxLidarCallback(livox_interfaces::msg::CustomMsg::SharedPtr livox_msg_in)
    {
        this->pc_last->clear();
        int plsize = livox_msg_in->point_num;
        if (plsize == 0) return;
        this->pc_last->reserve(plsize);
        pcl::PointXYZINormal pt;
        for (int i = 1; i < plsize; i++) {
            if ((livox_msg_in->points[i].tag & 0x30) == 0x10 || (livox_msg_in->points[i].tag & 0x30) == 0x00) {
                pt.x = livox_msg_in->points[i].x;
                pt.y = livox_msg_in->points[i].y;
                pt.z = livox_msg_in->points[i].z;
                pt.intensity = float (livox_msg_in->points[i].offset_time) / float (1e6); // unit: ms         
                pt.curvature = livox_msg_in->points[i].reflectivity;
                pc_last->points.push_back(pt);
            }
        }
        this->pc_last->header.frame_id = this->frame_id;
        this->pc_last->header.stamp = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*this->pc_last, *this->pc_last, indices);
        if (this->pc_last->points.empty()) return;
        ds_filter_each_scan.setInputCloud(pc_last);
        this->pc_last_ds->clear();
        ds_filter_each_scan.filter(*this->pc_last_ds);
        pc_last_ds->header.frame_id = this->frame_id;
        pc_last_ds->header.stamp = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        mtx.lock();
        this->pc_L_buff.push_back(*pc_last_ds);
        mtx.unlock();
    }

  private:
    rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr pc_subscription_livox;
};

class HesaiBuff : public MappingBase<pcl::PointXYZINormal>
{
  public:
  HesaiBuff(rclcpp::Node::SharedPtr &nh, const LidarConfig& lidar_config) : MappingBase<pcl::PointXYZINormal>(nh, lidar_config)
    {
        pc_subscription_hesai = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->lidar.topic, 100, std::bind(&HesaiBuff::hesaiLidarCallback, this, std::placeholders::_1));
    }

    void hesaiLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr hesai_msg_in)
    {
        this->pc_last->clear();
        pcl::PointCloud<hesai_ros::Point>::Ptr pc_last_hesai(new pcl::PointCloud<hesai_ros::Point>());
        pcl::fromROSMsg(*hesai_msg_in, *pc_last_hesai);
        size_t plsize = pc_last_hesai->size();
        if (plsize == 0) return;
        this->pc_last->reserve(plsize);
        rclcpp::Time timestamp_begin = rclcpp::Time(hesai_msg_in->header.stamp);
        pcl::PointXYZINormal pt;
        for (uint i = 0; i < plsize; i++) {
            pt.x = pc_last_hesai->points[i].x;
            pt.y = pc_last_hesai->points[i].y;
            pt.z = pc_last_hesai->points[i].z;
            double timestamp_s;
            double timestamp_ns = std::modf(pc_last_hesai->points[i].timestamp, &timestamp_s);
            rclcpp::Time timestamp_ros(static_cast<int32_t>(timestamp_s), static_cast<int32_t>(timestamp_ns * 1.0e9),
                rcl_clock_type_t::RCL_ROS_TIME);
            pt.intensity = (timestamp_ros - timestamp_begin).seconds() * 1.0e3; 
            pt.curvature = pc_last_hesai->points[i].intensity;

            if (pt.intensity >= 0) {
                this->pc_last->points.push_back(pt);
            }
        }
        this->pc_last->header.frame_id = this->frame_id;
        this->pc_last->header.stamp = rclcpp::Time(hesai_msg_in->header.stamp).nanoseconds();
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*this->pc_last, *this->pc_last, indices);
        if (this->pc_last->points.empty()) return;
        ds_filter_each_scan.setInputCloud(pc_last);
        this->pc_last_ds->clear();
        ds_filter_each_scan.filter(*this->pc_last_ds);
        pc_last_ds->header.frame_id = this->frame_id;
        pc_last_ds->header.stamp = rclcpp::Time(hesai_msg_in->header.stamp).nanoseconds();
        mtx.lock();
        this->pc_L_buff.push_back(*pc_last_ds);
        mtx.unlock();
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription_hesai;
};

class Mid360BoxiBuff : public MappingBase<pcl::PointXYZINormal>
{
  public:
  Mid360BoxiBuff(rclcpp::Node::SharedPtr &nh, const LidarConfig& lidar_config) : MappingBase<pcl::PointXYZINormal>(nh, lidar_config)
    {
        pc_subscription_mid360 = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->lidar.topic, 100, std::bind(&Mid360BoxiBuff::mid360BoxiCallback, this, std::placeholders::_1));
    }

    void mid360BoxiCallback(const sensor_msgs::msg::PointCloud2::SharedPtr livox_msg_in)
    {
        this->pc_last->clear();
        pcl::PointCloud<livox_mid360_boxi::Point>::Ptr pc_last_livox(new pcl::PointCloud<livox_mid360_boxi::Point>());
        pcl::fromROSMsg(*livox_msg_in, *pc_last_livox);
        size_t plsize = pc_last_livox->size();
        if (plsize == 0) return;
        this->pc_last->reserve(plsize);
        rclcpp::Time timestamp_begin = rclcpp::Time(livox_msg_in->header.stamp);
        pcl::PointXYZINormal pt;
        for (uint i = 0; i < plsize; i++) {
            pt.x = pc_last_livox->points[i].x;
            pt.y = pc_last_livox->points[i].y;
            pt.z = pc_last_livox->points[i].z;
            rclcpp::Time timestamp_ros(static_cast<int64_t>(pc_last_livox->points[i].timestamp),
                rcl_clock_type_t::RCL_ROS_TIME);
            pt.intensity = (timestamp_ros - timestamp_begin).seconds() * 1.0e3; 
            pt.curvature = pc_last_livox->points[i].intensity;

            if (pt.intensity >= 0) {
                this->pc_last->points.push_back(pt);
            }
        }
        this->pc_last->header.frame_id = this->frame_id;
        this->pc_last->header.stamp = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*this->pc_last, *this->pc_last, indices);
        if (this->pc_last->points.empty()) return;
        ds_filter_each_scan.setInputCloud(pc_last);
        this->pc_last_ds->clear();
        ds_filter_each_scan.filter(*this->pc_last_ds);
        pc_last_ds->header.frame_id = this->frame_id;
        pc_last_ds->header.stamp = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        mtx.lock();
        this->pc_L_buff.push_back(*pc_last_ds);
        mtx.unlock();
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription_mid360;
};

class Mapping
{

public:

Mapping(rclcpp::Node::SharedPtr &nh, std::vector<MappingBase<pcl::PointXYZINormal>*>& mappings)
    {
        sub_start = nh->create_subscription<std_msgs::msg::Int64>("/start_time", 100, std::bind(&Mapping::startCallBack, this, std::placeholders::_1));
        spl_window_st_ns = 0;
        sub_est = nh->create_subscription<estimate_msgs::msg::Estimate>("/est_window", 10000, std::bind(&Mapping::getEstCallback, this, std::placeholders::_1));
        pub_path = nh->create_publisher<nav_msgs::msg::Path>("traj_path", 100);
        pub_knots = nh->create_publisher<sensor_msgs::msg::PointCloud>("active_control_points",20);
        opt_old_path.header.frame_id = odom_id;
        vis_maps = mappings;
        pub_odom = nh->create_publisher<nav_msgs::msg::Odometry>("odometry", 500);
        br = std::make_shared<tf2_ros::TransformBroadcaster>(nh);
        if_save_results = CommonUtils::readParam<bool>(nh, "if_save_results", false);
        if (if_save_results) {
            std::string gt_topic = CommonUtils::readParam<std::string>(nh, "topic_gt");
            sub_gt = nh->create_subscription<mocap4r2_msgs::msg::RigidBodies>(gt_topic, 100, std::bind(&Mapping::getGTCallback, this, std::placeholders::_1));            
            res_file_path = CommonUtils::readParam<std::string>(nh, "res_file_path");            
            t_l_gt = CommonUtils::readVector3d(nh, "t_l_gt");
            std::vector<double> q_l_gt_v;
            q_l_gt_v = CommonUtils::readParam<std::vector<double>>(nh, "q_l_gt");
            q_l_gt = Eigen::Quaterniond(q_l_gt_v[0], q_l_gt_v[1], q_l_gt_v[2], q_l_gt_v[3]);
            srv_finalize = nh->create_service<std_srvs::srv::Empty>(
                "/if_finish", std::bind(&Mapping::finalizeService, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));              
        }
    }

    void lock_mappings() {
        for (auto vis_map : vis_maps) {
            vis_map->mtx.lock();
        }
    }

    void unlock_mappings() {
        for (auto vis_map : vis_maps) {
            vis_map->mtx.unlock();
        }
    }    

    void process() {
        rclcpp::Rate rate(20);
        int64_t num_knot = 0;
        while (!if_finished) {
            if (if_init_succeed && spline_global.numKnots() > num_knot) {
                lock_mappings();
                publishPath();
                displayControlPoints();
                pubOdom();
                num_knot = spline_global.numKnots();
                unlock_mappings();
            }
            if (!if_init_succeed) {
                rate.sleep();
                continue;
            }
            for (const auto vis_map : vis_maps) {
                vis_map->processScan(&spline_global, spl_window_st_ns);  
            }                                
        }
    }   

    void saveResults()
    {
        if (if_save_results) {
            Eigen::Quaterniond q_bl = vis_maps[0]->lidar.q_bl;
            Eigen::Vector3d t_bl = vis_maps[0]->lidar.t_bl;
            Eigen::aligned_vector<PoseData> est;
            for (const PoseData& gt_data : gt) {
                int64_t t_ns = gt_data.time_ns;
                if (t_ns >= spline_global.minTimeNs() && t_ns <= spline_global.maxTimeNs()) {
                    Eigen::Quaterniond q_wb;
                    spline_global.itpQuaternion(t_ns, &q_wb);
                    Eigen::Vector3d t_wb = spline_global.itpPosition(t_ns); 
                    Eigen::Quaterniond q_wl = q_wb * q_bl;
                    Eigen::Vector3d t_wl = q_wb * t_bl + t_wb;                    
                    Eigen::Quaterniond q_w_gt = q_wl * q_l_gt;
                    Eigen::Vector3d t_w_gt = q_wl * t_l_gt + t_wl;

                    PoseData itp_est;
                    itp_est.time_ns = t_ns;
                    itp_est.orient = q_w_gt;
                    itp_est.pos = t_w_gt;
                    est.push_back(itp_est);
                } 
            }
            CommonUtils::savePoseTUM(res_file_path, est);
        }
    }    

    void finalizeService(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>,
                        const std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        if_finished = true;
        rclcpp::shutdown();
    }        

private:
    std::string node_name = "Mapping";
    int64_t spl_window_st_ns;
    SplineState spline_global;
    rclcpp::Subscription<estimate_msgs::msg::Estimate>::SharedPtr sub_est;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_start;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    nav_msgs::msg::Path opt_old_path;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_knots;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
    std::vector<MappingBase<pcl::PointXYZINormal>*> vis_maps;
    const std::string frame_id = "body";
    const std::string odom_id = "world";
    std::shared_ptr<tf2_ros::TransformBroadcaster> br;
    bool if_init_succeed = false;
    std::mutex m_spline;    

    bool if_save_results = false;
    bool if_finished = false;
    rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr sub_gt;    
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_finalize;
    std::string res_file_path;
    Eigen::aligned_vector<PoseData> gt;
    Eigen::Quaterniond q_l_gt;
    Eigen::Vector3d t_l_gt;      

    void displayControlPoints()
    {
        sensor_msgs::msg::PointCloud points_msg;
        points_msg.header.frame_id = odom_id;
        points_msg.header.stamp = rclcpp::Time(spline_global.minTimeNs());
        for (int64_t i = spline_global.numKnots() - 4; i < spline_global.numKnots(); i++) {
            points_msg.points.push_back(CommonUtils::getPointMsg(spline_global.getKnotPos(i)));
        }
        pub_knots->publish(points_msg);
    }

    void getEstCallback(const estimate_msgs::msg::Estimate::SharedPtr est_msg)
    {
        if (!if_init_succeed) {
            return;
        }
        estimate_msgs::msg::Spline spline_msg = est_msg->spline;
        SplineState spline_w;
        
        spline_w.init(spline_msg.dt, 0, spline_msg.start_t, spline_msg.start_idx);
        for(const auto& knot : spline_msg.knots) {
            Eigen::Vector3d pos(knot.position.x, knot.position.y, knot.position.z);
            Eigen::Vector3d quat_del(knot.orientation_del.x, knot.orientation_del.y, knot.orientation_del.z);
            spline_w.addOneStateKnot(pos, quat_del);
        }
        Eigen::Quaterniond q_idle0 = Eigen::Quaterniond(spline_msg.start_q.w, spline_msg.start_q.x, spline_msg.start_q.y, spline_msg.start_q.z);
        for (int i = 0; i < 3; i++) {
            estimate_msgs::msg::Knot idle = spline_msg.idles[i];
            Eigen::Vector3d t_idle(idle.position.x, idle.position.y, idle.position.z);
            Eigen::Vector3d quat_idle(idle.orientation_del.x, idle.orientation_del.y, idle.orientation_del.z);
            spline_w.setIdles(i, t_idle, quat_idle, q_idle0);
        }
        lock_mappings();
        spl_window_st_ns = spline_msg.start_t - spline_msg.dt; 
        spline_global.setTimeIntervalNs(spline_msg.dt);
        spline_global.updateKnots(&spline_w);
        unlock_mappings();
    }

    void getGTCallback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg_in)
    {
        int idx_gt = 4;
        const auto& gt_pose = msg_in->rigidbodies[idx_gt].pose;
        if (isnan(gt_pose.position.x)) {
            return;
        }
        Eigen::Quaterniond orient_gt = Eigen::Quaterniond(gt_pose.orientation.w, gt_pose.orientation.x, gt_pose.orientation.y, gt_pose.orientation.z);
        Eigen::Vector3d pos_gt = Eigen::Vector3d(gt_pose.position.x, gt_pose.position.y, gt_pose.position.z);
        int64_t time_ns = rclcpp::Time(msg_in->header.stamp).nanoseconds();
        PoseData pose(time_ns, orient_gt, pos_gt);
        gt.push_back(pose);
    }

    void pubOdom()
    {
        if (opt_old_path.poses.empty()) {
            return;
        }
        nav_msgs::msg::Odometry odom_msg;
        geometry_msgs::msg::PoseStamped odom_pose = opt_old_path.poses.back();
        odom_msg.header.stamp = rclcpp::Time(odom_pose.header.stamp);
        odom_msg.header.frame_id = odom_id;
        odom_msg.child_frame_id = frame_id;
        odom_msg.pose.pose = odom_pose.pose;
        pub_odom->publish(odom_msg);      
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = odom_msg.header.stamp;
        transformStamped.header.frame_id = odom_id;
        transformStamped.child_frame_id = frame_id;
        transformStamped.transform.translation.x = odom_pose.pose.position.x;
        transformStamped.transform.translation.y = odom_pose.pose.position.y;
        transformStamped.transform.translation.z = odom_pose.pose.position.z;
        transformStamped.transform.rotation = odom_pose.pose.orientation;
        br->sendTransform(transformStamped);
        transformStamped.header.stamp = odom_msg.header.stamp;
        transformStamped.header.frame_id = odom_id;
        transformStamped.child_frame_id = "imu_frame";
        transformStamped.transform.translation.x = odom_pose.pose.position.x;
        transformStamped.transform.translation.y = odom_pose.pose.position.y;
        transformStamped.transform.translation.z = odom_pose.pose.position.z;
        transformStamped.transform.rotation.w = odom_pose.pose.orientation.w;
        transformStamped.transform.rotation.x = odom_pose.pose.orientation.x;
        transformStamped.transform.rotation.y = odom_pose.pose.orientation.y;
        transformStamped.transform.rotation.z = odom_pose.pose.orientation.z;
        br->sendTransform(transformStamped);

        int id_lidar = 0;
        for (const auto vis_map : vis_maps) {
            Eigen::Quaterniond q_wi(odom_pose.pose.orientation.w, odom_pose.pose.orientation.x, odom_pose.pose.orientation.y, odom_pose.pose.orientation.z);
            Eigen::Vector3d t_wi(odom_pose.pose.position.x, odom_pose.pose.position.y, odom_pose.pose.position.z);
            Eigen::Quaterniond q_wl = q_wi * vis_map->lidar.q_bl;
            Eigen::Vector3d t_wl = q_wi * vis_map->lidar.t_bl + t_wi;
            std::string lidar_str = "lidar" + std::to_string(id_lidar) + "_frame";
            transformStamped.header.stamp = odom_msg.header.stamp;
            transformStamped.header.frame_id = odom_id;
            transformStamped.child_frame_id = lidar_str;
            transformStamped.transform.translation.x = t_wl.x();
            transformStamped.transform.translation.y = t_wl.y();
            transformStamped.transform.translation.z = t_wl.z();
            transformStamped.transform.rotation.w = q_wl.w();
            transformStamped.transform.rotation.x = q_wl.x();
            transformStamped.transform.rotation.y = q_wl.y();
            transformStamped.transform.rotation.z = q_wl.z();
            br->sendTransform(transformStamped);
            id_lidar++;
        }                 
    }

    void startCallBack(const std_msgs::msg::Int64::SharedPtr start_time_msg)
    {
        int64_t bag_start_time = start_time_msg->data;
        spline_global.init(0, 0, bag_start_time, 0);
        if_init_succeed = true;
    }

    void publishPath() {
        if (!if_init_succeed || spline_global.numKnots() <= 4) {
            return;
        }
        static int64_t t_ns = spline_global.minTimeNs();
        while (t_ns < std::min(spl_window_st_ns, spline_global.maxTimeNs())) {
            Eigen::Quaterniond orient_interp;
            Eigen::Vector3d t_interp = spline_global.itpPosition(t_ns);
            spline_global.itpQuaternion(t_ns, &orient_interp);
            opt_old_path.poses.push_back(CommonUtils::pose2msg(t_ns, t_interp, orient_interp));
            t_ns += 1e8;
        }
        pub_path->publish(opt_old_path);
    }         

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("Mapping");
    std::vector<LidarConfig> lidars;
    auto lidar_names = nh->declare_parameter<std::vector<std::string>>("lidars", std::vector<std::string>());
    assert(nh->get_parameter({"lidars"}, lidar_names));
    if (lidar_names.empty()) {
        lidars.emplace_back(nh, "");
    } else {
        for (const auto& lidar_name : lidar_names) {
            lidars.emplace_back(nh, lidar_name + ".");
        }
    }
    std::vector<MappingBase<pcl::PointXYZINormal>*> buffs;
    for (const auto& lidar : lidars) {
        if (!lidar.type.compare("Ouster")) {
            buffs.push_back(new OusterBuff(nh, lidar));
        } else if (!lidar.type.compare("Mid70Avia")) {
            buffs.push_back(new Mid70AviaBuff(nh, lidar));
        } else if (!lidar.type.compare("HAP360")) {
            buffs.push_back(new HAP360Buff(nh, lidar));    
        } else if (!lidar.type.compare("AviaResple")) {
            buffs.push_back(new AviaRespleBuff(nh, lidar));
        } else if (!lidar.type.compare("Hesai")) {
            buffs.push_back(new HesaiBuff(nh, lidar));
        } else if (!lidar.type.compare("Mid360Boxi")) {
            buffs.push_back(new Mid360BoxiBuff(nh, lidar));
        } else {
            exit(1);
        }
    }
    Mapping mapping(nh, buffs);
    std::thread mappingThread{&Mapping::process, &mapping};
    rclcpp::spin(nh);
    mapping.saveResults();
    mappingThread.join();
    rclcpp::shutdown();
}