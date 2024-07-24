// #include <memory>
// #include <chrono>


#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"
#include "iss_loam/utils/common.hpp"




// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <std_msgs/msg/int64.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
// #include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

// #include "example_interfaces/srv/add_two_ints.hpp"
// #include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/correspondence.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/convergence_criteria.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
// #include <pcl/registration/transformation_estimation_3point.h>
#include <pcl/registration/ndt.h>
// #include <fast_pcl/registration/ndt.h>
// #include <fast_pcl/registration/ndt.h>

// #include <pclomp/gicp_omp.h>
// #include <pclomp/gicp_omp_impl.hpp>
// #include <pclomp/ndt_omp.h>
// #include <pclomp/ndt_omp_impl.hpp>
// #include <pclomp/voxel_grid_covariance_omp.h>
// #include <pclomp/voxel_grid_covariance_omp_impl.hpp>


#include <cmath>
#include <ctime>
// #include <array>
// #include <string>
#include <vector>
// #include <algorithm>
#include <iostream>
#include <fstream>
// #include <thread>
// #include <mutex>
#include <queue>
// #include <assert.h>
#include <iomanip>
// #include <ctime>
#include <sstream>

#define _USE_MATH_DEFINES


///////////////////////////////////////////////////////////////////////////////////////////

class INS //: public rclcpp::Node
{
public:
    INS()
    // : Node("ins")
    {  
        preint_state.ori = Eigen::Quaterniond::Identity();
        preint_state.pos = Eigen::Vector3d::Zero();
        preint_state.vel = Eigen::Vector3d::Zero();
        preint_state.acc = Eigen::Vector3d::Zero();
        preint_state.time = 0.0;

        current_bias.acc = Eigen::Vector3d::Zero();
        current_bias.ang = Eigen::Vector3d::Zero();
        current_bias.time = 0.0;

        preint_state.bias = current_bias;

        preint_transformation_guess.rotation = Eigen::Quaterniond::Identity();
        preint_transformation_guess.translation = Eigen::Vector3d::Zero();
        preint_transformation_guess.matrix = Eigen::Matrix4d::Identity();
        preint_transformation_guess.time = 0.0;
    }
    ~INS(){}

private:
    Pose initial_pose;
    Pose current_pose;

    // double expected_imu_dt = 0.005;

    INSstate preint_anchor;
    INSstate preint_state;
    INSstate predict_state;
    INSstate lidar_synced_state;

    deque<INSstate> preint_states;
    deque<INSstate> predict_states;

    Transformation preint_transformation_guess;

    IMUwrench current_bias;


    deque<double> imu_dts;
    double mean_imu_dt = 0.005;
    double std_imu_dt = 0.02;
    double last_imu_stamp{};

    deque<sensor_msgs::msg::Imu::SharedPtr> imu_msg_buffer;

    Eigen::Quaterniond madgwickOrientation = Eigen::Quaterniond::Identity();
    
    
public:
    void addImuMsgToBuffer(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    const sensor_msgs::msg::Imu::SharedPtr getNextMsgFromBuffer(void);
    void resetPreintAnchor();
    void setPreintAnchor(const INSstate state);
    void getPreintAnchor(INSstate &state); 
    INSstate getPreintAnchor(); 
    void setBias(const IMUwrench new_bias);

    void initializeInitialPose(Eigen::Quaterniond);

    Transformation& getPreintegratedTransformationGuess(); 

    void updateInitialPose(const Pose initial_pose);
    
    double getIMUdt(int buffer_index);
    
    void updateIMUdtAverage(double new_dt);
    
    double validateIMUdt(double new_dt);
    
    void updateOrientationFromAngularRate(Eigen::Quaterniond &orientation, Eigen::Vector3d angular_rate, double dt);

    void integrateImuStep(Eigen::Vector3d acc, Eigen::Vector3d ang_vel, double dt);

    void integrateConstVelocityStep(double dt);

    void integrateJerkAlpha(INSstate *this_state, const INSstate next_state, double dt);

    void  integrateJerkAlphaBackwards(INSstate *this_state, const INSstate previous_state);

    IMUwrench createIMUwrench(Eigen::Vector3d acc, Eigen::Vector3d ang, double dt, double t);

    void preintegrateIMU(const double last_scan_start_time, const double end_time);

    void predictConstVelocityStep(double dt);
    void predictionIntegrate(double deltaT, int steps);

    void calcLidarSyncedState(const double frame_start_time);

    INSstate calcKinematicInterpolatedState(const INSstate & this_state, const INSstate & next_state, double interpolate_time);

    void undistortCloud(const pcl::PointCloud<PointType> &cloud_in, pcl::PointCloud<PointType> &cloud_out);

    const INSstate& getPreintState(int index);
    const INSstate& getPredictState(int index);


    Eigen::Quaterniond getMadgwickOrientation(double timestamp);

};

void INS::addImuMsgToBuffer(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{   
    this->imu_msg_buffer.push_back(imu_msg);
    updateIMUdtAverage(toSec(imu_msg->header.stamp) - last_imu_stamp);
    last_imu_stamp = toSec(imu_msg->header.stamp);
}

const sensor_msgs::msg::Imu::SharedPtr INS::getNextMsgFromBuffer(void)
{
    auto msg = this->imu_msg_buffer.front();
    this->imu_msg_buffer.pop_front();
    return msg;
}

void INS::resetPreintAnchor()
{
    preint_anchor.pos   = Eigen::Vector3d::Zero(); // initial position in the future
    preint_anchor.vel   = Eigen::Vector3d::Zero(); // saved in local frame
    preint_anchor.ori   = Eigen::Quaterniond(initial_pose.orientation);
    preint_anchor.ang   = Eigen::Vector3d::Zero();
    // preint_anchor.bias = Eigen::Vector3d::Zero();
    preint_anchor.acc   = Eigen::Vector3d::Zero();
    preint_anchor.jerk  = Eigen::Vector3d::Zero();
    preint_anchor.alpha = Eigen::Vector3d::Zero();
    // preint_anchor.bias  = IMUwrench();

}

void INS::setPreintAnchor(const INSstate state)
{
    
    preint_anchor.time  = state.time ;
    preint_anchor.pos   = state.pos ;
    preint_anchor.vel  = state.vel ;
    // preint_anchor.vel   = state.ori.matrix().inverse() * state.vel ; // velocity saved in local frame, so it can rotated into a corrected frame
    preint_anchor.ori   = state.ori ;
    preint_anchor.ang   = state.ang ;
    preint_anchor.bias  = state.bias;
    preint_anchor.acc   = state.acc ;
    preint_anchor.jerk  = state.jerk ;
    preint_anchor.alpha = state.alpha ;
    preint_anchor.dt    = state.dt;
}

// set to return anchor by reference?
void INS::getPreintAnchor(INSstate &state) // passed by reference to make it mutable
{
   
    state.time  = preint_anchor.time;
    state.pos   = preint_anchor.pos ;
    state.vel   = preint_anchor.vel; 
    // state.vel   = state.ori.matrix() * preint_anchor.vel; // is rotated into new frame
    state.ori   = preint_anchor.ori ;
    state.ang   = preint_anchor.ang ;
    state.bias  = preint_anchor.bias;
    state.acc   = preint_anchor.acc ;
    state.jerk  = preint_anchor.jerk ;
    state.alpha = preint_anchor.alpha ;
    state.dt    = preint_anchor.dt;
}

INSstate INS::getPreintAnchor() 
{
    INSstate state;
    state.time  = preint_anchor.time;
    state.pos   = preint_anchor.pos ;
    // state.vel   = state.ori.matrix() * preint_anchor.vel; // is rotated into new frame
    state.vel   = preint_anchor.vel; 
    state.ori   = preint_anchor.ori ;
    state.ang   = preint_anchor.ang ;
    state.bias  = preint_anchor.bias;
    state.acc   = preint_anchor.acc ;
    state.jerk  = preint_anchor.jerk ;
    state.alpha = preint_anchor.alpha ;
    state.dt    = preint_anchor.dt;
    return state;
}

void INS::setBias(const IMUwrench new_bias)
{
    this->current_bias = new_bias;
}

void INS::initializeInitialPose(Eigen::Quaterniond orientation)
{
    initial_pose.orientation             =  Eigen::Quaterniond(orientation);
    preint_state.ori                     =  Eigen::Quaterniond(orientation);
    preint_anchor.ori                    =  Eigen::Quaterniond(orientation);
    preint_transformation_guess.rotation =  Eigen::Quaterniond(orientation);
    preint_transformation_guess.matrix.block<3,3>(0,0) = Eigen::Matrix3d(orientation);
    // cout << "FROM INS: Initial orientation :\n" << orientation.w() <<"\n"<< orientation.vec() << "\n";

}

Transformation& INS::getPreintegratedTransformationGuess(){
    // Transformation preint_transformation_guess;
    // preint_transformation_guess.translation = lidar_synced_state.pos;
    // preint_transformation_guess.rotation = lidar_synced_state.ori;
    return this->preint_transformation_guess;
}

void INS::updateInitialPose(const Pose initial_pose)
{
    // make something that averages the incoming poses??
    this->initial_pose = initial_pose;
    this->current_pose = initial_pose;
    // std::cout << "FROM INS: initial pose updated q: " << std::to_string(this->initial_pose.orientation.w()) << " " << std::to_string(this->initial_pose.orientation.x()) <<" " << std::to_string(this->initial_pose.orientation.y()) <<" " << std::to_string(this->initial_pose.orientation.z()) << "\n";
}

double INS::getIMUdt(int buffer_index)
{
    double dt = toSec(imu_msg_buffer[buffer_index+1]->header.stamp) - toSec(imu_msg_buffer[buffer_index]->header.stamp);
    return dt;
    
}

void INS::updateIMUdtAverage(double new_dt)
{   
    // cout << "new imu dt "<< to_string(new_dt) << " mean is: "<< to_string(mean_imu_dt) <<" std is: "<< to_string(std_imu_dt) << "\n";
    // if (abs(new_dt - mean_imu_dt) < 3.0*std_imu_dt && std_imu_dt > mean_imu_dt/5.0){
    if (abs(new_dt - mean_imu_dt) < 3.0*std_imu_dt){
        // cout << "imu dt sample added"<< "\n";
        if (this->imu_dts.size() > 1000){ // limit the size of buffer else mean calculations would slowly increase in time
            imu_dts.pop_back();
        }
        imu_dts.push_front(new_dt);
    } else {
        return;
    }

    // cout << "FROM INS: imu msg dt: " << to_string(new_dt) << "\n";
    // TODO make a running mean calculation instead
    size_t n = imu_dts.size();
    if (n >= 10){
        double sum = 0.0;
        double sum_sq = 0.0;
        for (size_t i=0; i < n; i++){
            sum += imu_dts[i];
            sum_sq += imu_dts[i]*imu_dts[i];
        }
        mean_imu_dt = sum / (double)n;
        std_imu_dt = sqrt((sum_sq)/(double)n - mean_imu_dt*mean_imu_dt );
        // cout << "new mean is: "<< to_string(mean_imu_dt) <<" new std is: "<< to_string(std_imu_dt) << "\n";
    } 
    return;

}

// should add an expected dt?
double INS::validateIMUdt(double new_dt)
{   
    // RCLCPP_INFO(get_logger(),"IMU dt: %f", new_dt);
    // return expected_imu_dt;
    if (this->imu_dts.size() < 10){ // need atleast a little sample to do mean calculations
        // cout << "not enough imu dt samples to validate imu"<< "\n";
        return new_dt;
    }

    // if (abs(new_dt - mean_dt) > 2.0*std_dt){ // if the time deviation is by too big it is replaced by the mean of the good samples
    if (new_dt - mean_imu_dt > 3.0*std_imu_dt){ // if the time deviation is by too big it is replaced by the mean of the good samples
        // cout << "FROM INS: Bad dt detected! " << to_string(new_dt) << " replaced with mean " << to_string(mean_imu_dt) << " std " << to_string(std_imu_dt) <<"\n";
        return mean_imu_dt;
    }
    
    // past here, the dt values is good and is added to good samples and is returned as it is
    // cout << "good imu dt"<< "\n";
    return new_dt;
}

void INS::updateOrientationFromAngularRate(Eigen::Quaterniond &orientation, Eigen::Vector3d angular_rate, double dt)
{
    Eigen::Quaterniond dq_vel = deltaQ(angular_rate); // the 0.5 is to find the average
    dq_vel = multQuatByScalar(dq_vel, dt);
    Eigen::Quaterniond dq = orientation * dq_vel; // 1/2 * q * dq * dt
    orientation = addQuaternions(dq, preint_state.ori).normalized(); 
}

// OBS!: This function assumes that gravity has been removed from the incoming acceleration.
void INS::integrateImuStep(Eigen::Vector3d acc, Eigen::Vector3d ang_vel, double dt)
{
    // save the previous state to be used 
    INSstate previous_state = preint_state;

    // Integrate the orientation from the angular rate and save the average ori between this and the previous state
    preint_state.ang = ang_vel - current_bias.ang;
    // Eigen::Quaterniond dq_vel = deltaQ(0.5*(preint_state.ang + previous_state.ang)); // the 0.5 is to find the average
    // dq_vel = multQuatByScalar(dq_vel, dt);

    // Eigen::Quaterniond dq = preint_state.ori*dq_vel; // 1/2 * q * dq * dt
    // preint_state.ori = addQuaternions(dq, preint_state.ori).normalized(); 

    updateOrientationFromAngularRate(preint_state.ori, 0.5*(preint_state.ang + previous_state.ang), dt );

    // preint_state.ori.normalize(); // orientation normalized
    Eigen::Quaterniond ori_avg = previous_state.ori.slerp(0.5, preint_state.ori);

    // need to think about how bias is applied and saved..
    // acceleration and bias is in the local imu frame
    // preint_state.acc = (acc - observer_state.bias.acc); // save the acc in local body frame - with subtracted bias?
    preint_state.acc = ((acc - current_bias.acc) + previous_state.acc) / 2.0; // save the acc in local body frame ass the average of the previous and the input- with subtracted bias

    preint_state.jerk = (preint_state.ori.matrix() * preint_state.acc - previous_state.ori.matrix() * previous_state.acc) / dt; // save the acc in local body frame - with subtracted bias
    preint_state.alpha = (preint_state.ang - previous_state.ang) / dt; // save the acc in local body frame - with subtracted bias

    preint_state.vel = previous_state.vel + ori_avg.matrix() * preint_state.acc * dt;
    preint_state.pos = previous_state.pos + preint_state.vel * dt - ori_avg.matrix() * preint_state.acc * dt*dt *0.5;
    
    // preint_state.dt = previous_state.time + dt;
    // preint_state.time = previous_state.time + dt;
    ///velocity in local frame:
    // preint_state.vel = previous_state.vel +  preint_state.acc * dt; 
    // preint_state.pos = previous_state.pos + ori_avg.matrix() * preint_state.vel * dt - ori_avg.matrix() * preint_state.acc * dt*dt *0.5;
}

void INS::integrateConstVelocityStep(double dt)
{
    // save the previous state to be used 
    INSstate previous_state = preint_state;

    // Integrate the orientation from the angular rate and save the average ori between this and the previous state
    preint_state.ang = Eigen::Vector3d::Zero(); //previous_state.ang; // the angular rate is assumed constant and the previous measurement is used.
    Eigen::Quaterniond dq_vel = deltaQ(0.5*(preint_state.ang + previous_state.ang)); 
 
    dq_vel = multQuatByScalar(dq_vel, dt);

    Eigen::Quaterniond dq = preint_state.ori*dq_vel;
    preint_state.ori = addQuaternions(dq, preint_state.ori);
    preint_state.ori.normalize(); // orientation normalized
    // Eigen::Quaterniond ori_avg = previous_state.ori.slerp(0.5, preint_state.ori);

    preint_state.acc = Eigen::Vector3d::Zero(); // net acceleration is assumed zero - ofc not true while moving in a curve..
    preint_state.vel = previous_state.vel; 
    preint_state.pos = previous_state.pos + preint_state.vel * dt;

    // velocity in local frame
    // preint_state.pos = previous_state.pos + ori_avg.matrix() * preint_state.vel * dt;  //- ori_avg.matrix() * preint_state.acc * dt*dt *0.5;

    // jerk and alpha is zero in a constant velocity model
    preint_state.jerk = Eigen::Vector3d::Zero(); 
    preint_state.alpha = Eigen::Vector3d::Zero();
}

void INS::integrateJerkAlpha(INSstate *this_state, const INSstate next_state, double dt)
{
    // Eigen::Quaterniond ori_avg = this_state.ori.slerp(0.5, next_state.ori);
    this_state->alpha = (next_state.ang - this_state->ang) / dt;
    this_state->jerk =  (next_state.ori*next_state.acc - this_state->ori*this_state->acc) / dt; // calculate and save jerk in world frame
    // this_state->jerk =  this_state->ori.inverse() * (next_state.ori*next_state.acc - this_state->ori*this_state->acc) / dt;
}

void INS::integrateJerkAlphaBackwards(INSstate *this_state, const INSstate previous_state)
{
    // Eigen::Quaterniond ori_avg = this_state.ori.slerp(0.5, next_state.ori);
    this_state->alpha = (this_state->ang - previous_state.ang) / previous_state.dt;
    this_state->jerk =  (this_state->ori*this_state->acc - previous_state.ori*previous_state.acc) / previous_state.dt; // calculate and save jerk in world frame
    // this_state->jerk =  this_state->ori.inverse() * (next_state.ori*next_state.acc - this_state->ori*this_state->acc) / dt;
}

IMUwrench INS::createIMUwrench(Eigen::Vector3d acc, Eigen::Vector3d ang, double dt, double t)
{
    IMUwrench new_imu_wrench;
    new_imu_wrench.acc = acc;
    new_imu_wrench.ang = ang;
    new_imu_wrench.time = t;
    new_imu_wrench.dt = dt;
    return new_imu_wrench;
}

void INS::preintegrateIMU(const double last_scan_start_time, const double next_scan_end_time)
{

    // deque<sensor_msgs::msg::Imu::SharedPtr> preintegrate_buffer;
    deque<IMUwrench> preintegrate_buffer;
    // the total time delta that the preintegration should cover
    double integration_delta = next_scan_end_time - last_scan_start_time;

    cout << "FROM INS: Integration delta: " << integration_delta << ", start: " << to_string(last_scan_start_time) << " end: " << to_string(next_scan_end_time) << "\n";
    // double integration_time = 0.0;
    // clear out buffer up till the first imu msg before the scan begins, the anchor state should be just before the start of the scan
    double imu_msg_dt{};
    double imu_dt{};
    // double lag_discrepancy{};

    // removes past imu msg 
    while(last_scan_start_time > toSec(imu_msg_buffer[1]->header.stamp) ) // >=?
    {
        imu_msg_buffer.pop_front();
    }

    int j = 0;
    double msg_time = toSec(imu_msg_buffer[0]->header.stamp);
    double delay = last_scan_start_time - msg_time;
    if (delay > mean_imu_dt){ // if the time difference is bigger than the expected imu delta then add a msg at the start time of the frame
        // cout << "delay!: "<< delay << "\n";
        j++;
        Eigen::Vector3d acc_in(0.0,0.0,0.0);
        Eigen::Vector3d ang_vel_in(0.0,0.0,0.0);
        double fill_time = toSec(imu_msg_buffer[1]->header.stamp) - last_scan_start_time ;
        IMUwrench new_imu_wrench = createIMUwrench(acc_in, ang_vel_in, fill_time, last_scan_start_time);    
        preintegrate_buffer.push_back(new_imu_wrench);
        msg_time = last_scan_start_time;
        // cout << "FROM INS: !const vel! msg added " << j << "\ttimestamp: " << to_string(last_scan_start_time) << "\tmsg dt: "<< fill_time << "\tdt: " << mean_imu_dt << "\n";
    }

    sensor_msgs::msg::Imu imu_msg;
    // while (true){
    while (msg_time <= next_scan_end_time){
        imu_msg = *imu_msg_buffer[j];
        msg_time = toSec(imu_msg.header.stamp);
        imu_msg_dt = getIMUdt(j); 
        imu_dt = validateIMUdt(imu_msg_dt); 
        // lag_discrepancy = imu_msg_dt - imu_dt;


        // extract acc and ang vel from the imu msg
        Eigen::Vector3d acc_in(imu_msg.linear_acceleration.x,
                               imu_msg.linear_acceleration.y,
                               imu_msg.linear_acceleration.z);
        Eigen::Vector3d ang_vel_in(imu_msg.angular_velocity.x,
                                   imu_msg.angular_velocity.y,
                                   imu_msg.angular_velocity.z);
        
        IMUwrench new_imu_wrench = createIMUwrench(acc_in, ang_vel_in, imu_msg_dt, msg_time);    
        preintegrate_buffer.push_back(new_imu_wrench);
        j++;
        // cout << "FROM INS: new msg added" << j << "\ttimestamp: " << to_string(msg_time) << "\tmsg dt:"<< imu_msg_dt << "\tdt: " << imu_dt << "\n";
        // cout << " acc: " << acc_in.transpose() << "\n ang:  " << ang_vel_in.transpose() << "\n"; 
    }

    cout << "FROM INS: preintegrate buffer length: " << preintegrate_buffer.size() << "\n";
    
    // get the preintegration achor state, this is the state that the integration iterates from
    getPreintAnchor(preint_state);
    // overwrite timestamp of the anchor to the correct time of the next msg? no
    // preint_state.time = preintegrate_buffer.front().time;
    preint_states.clear(); // clear preint_states to make sure it is empty

    INSstate this_preint_state = preint_state;
    INSstate new_preint_state;
    
    preint_states.push_back(this_preint_state); // push the anchor state
    
    // cout << "FROM INS: before integration preint state rotation:\n" << preint_state.ori.w() <<"\n"<< preint_state.ori.vec() << "\n";
    // integrate all the collected imu msg in the integration delta time
    for (size_t i = 0; i < preintegrate_buffer.size(); i++) {
        IMUwrench imu_input = preintegrate_buffer[i];
        // double msg_time = imu_input.time;
        // imu_dt = validateIMUdt(imu_input.dt); // this doesnt work 
        imu_dt = imu_input.dt;

        integrateImuStep(imu_input.acc, imu_input.ang, imu_dt); // this writes to preint_state, but does not calc jerk and alpha
        // cout << "FROM INS: after integration preint state rotation:\n" << preint_state.ori.w() <<"\n"<< preint_state.ori.vec() << "\n";
        new_preint_state = preint_state;
        integrateJerkAlpha(&this_preint_state, new_preint_state, imu_dt); // calcs jerk and alpha between this and next and saves to this_state
        // this_preint_state.time = toSec(imu_msg.header.stamp);
        this_preint_state.time = imu_input.time;
        this_preint_state.dt = imu_dt;
        preint_states.push_back(this_preint_state);
        preint_state = new_preint_state; // this is not needed i think
        this_preint_state = new_preint_state;





        // integrateImuStep(imu_input.acc, imu_input.ang, imu_dt); // this writes to preint_state, but does not calc jerk and alpha
        // new_preint_state = preint_state;
        // // integrateJerkAlpha(&this_preint_state, new_preint_state, this_preint_state.dt); // calcs jerk and alpha between this and next and saves to this_state
        // // integrateJerkAlphaBackwards(&preint_state, this_preint_state); // calcs jerk and alpha between this and next and saves to this_state

        // // this_preint_state.time = toSec(imu_msg.header.stamp);
        // new_preint_state.time = imu_input.time;
        // new_preint_state.dt = imu_dt;
        // preint_states.push_back(new_preint_state);
        // // preint_state = this_preint_state;
        // this_preint_state = new_preint_state;


        // integrateImuStep(imu_input.acc, imu_input.ang, imu_dt); // this writes to preint_state, but does not calc jerk and alpha
        // // cout << "FROM INS: after integration preint state rotation:\n" << preint_state.ori.w() <<"\n"<< preint_state.ori.vec() << "\n";
        // integrateJerkAlphaBackwards(&preint_state, this_preint_state); // calcs jerk and alpha between this and next and saves to this_state
        // // this_preint_state.time = toSec(imu_msg.header.stamp);
        // // new_preint_state = preint_state;
        // preint_state.time = imu_input.time;
        // preint_state.dt = imu_dt;
        // this_preint_state = preint_state;
        // preint_states.push_back(this_preint_state);
        // // preint_state = new_preint_state;
        // // this_preint_state = new_preint_state;

        // integration_time += imu_dt;

        // Eigen::Quaterniond rotation = preint_state.ori.inverse() * preint_anchor.ori; 
        // Eigen::Vector3d translation = preint_state.pos - preint_anchor.pos;

        // // double preint_norm = translation.norm();
        // Eigen::AngleAxisd angleAxis = Eigen::AngleAxisd(rotation);
        // double preint_angle = angleAxis.angle()*180.0/M_PI;

        // cout << "idx: "<< i << " timestamp: " << to_string(msg_time) << " msg dt: "<< imu_input.dt << " dt: " << imu_dt << "\n";
        // cout << "acc: " << imu_input.acc.transpose() << " ang:  " << imu_input.ang.transpose() << "\n";
        // cout << "preintegrated:"<< i << "\t translation:" << translation.transpose() << " m,\tangle: " << preint_angle << " degrees \n";

        // if (msg_time > next_scan_end_time){ // not really need anymore
        // // if (integration_time > integration_delta){ 
        //     return;
        // }
    }
} 

void INS::predictConstVelocityStep(double dt)
{
    // save the previous state to be used 
    INSstate previous_state = predict_state;

    // Integrate the orientation from the angular rate and save the average ori between this and the previous state
    predict_state.ang = previous_state.ang;
    Eigen::Quaterniond dq_vel = deltaQ(0.5*(predict_state.ang + previous_state.ang)); 
 
    dq_vel = multQuatByScalar(dq_vel, dt);

    Eigen::Quaterniond dq = predict_state.ori*dq_vel;
    predict_state.ori = addQuaternions(dq, predict_state.ori); // save new orientation normalized!
    predict_state.ori.normalize(); // orientation normalized!
    Eigen::Quaterniond ori_avg = previous_state.ori.slerp(0.5, predict_state.ori);

    // need to think about how bias is applied and saved..
    // acceleration and bias is in the local imu frame
    predict_state.acc = previous_state.acc; // maybe assume constant acceleration? this is only used for vizualisation of the future trajectory 
    // predict_state.acc = Eigen::Vector3d::Zero();
    predict_state.vel = previous_state.vel + ori_avg.matrix() * predict_state.acc * dt; 
    // predict_state.vel = previous_state.vel; // + predict_state.acc * dt; 
    predict_state.pos = previous_state.pos + predict_state.vel * dt - ori_avg.matrix() * predict_state.acc * dt*dt *0.5;
    // predict_state.pos = previous_state.pos + ori_avg.matrix() * predict_state.vel * dt;

    predict_state.jerk = Eigen::Vector3d::Zero();
    predict_state.alpha = Eigen::Vector3d::Zero();
}

void INS::predictionIntegrate(double dt, int steps)
{

    predict_state = preint_anchor;
    INSstate this_predict_state = preint_anchor;
    INSstate new_predict_state;

    predict_states.clear(); // clear predict_states to make sure it is empty
    for (int i = 0; i < steps; i++){

            predictConstVelocityStep(dt); // this writes to preint_state, but does not calc jerk and alpha
            new_predict_state = predict_state;
            this_predict_state.time = predict_state.time + i*dt;
            this_predict_state.dt = dt;
            predict_states.push_back(this_predict_state);
            predict_state = new_predict_state;
            this_predict_state = new_predict_state;

    }
} 

void INS::calcLidarSyncedState(const double frame_start_time)
{
    // get the latest integrated state just before the start of the new cloud scan
    // size_t i = 0;
    while (preint_states[1].time <= frame_start_time) // comparing to 1 to get the state just before frame_start
    {
        preint_states.pop_front();
    }

    // set state anchor for next preintegration - this is synced with imu steps
    // setPreintAnchor(preint_states[i]);
    setPreintAnchor(preint_states.front()); // THIS SHOULD STILL BE THE CORRECT THING TO USE

    // calculates the imu syncronised interpolated state at time of the start of the cloud
    double sync_time = frame_start_time - preint_states.front().time;
   
    lidar_synced_state = calcKinematicInterpolatedState(preint_states[0], preint_states[1], sync_time);
    // setPreintAnchor(lidar_synced_state);

    // this synced state is also the prior for the scan registration 
    // it is saved so it can be used in the observer
    preint_transformation_guess.translation = lidar_synced_state.pos;
    preint_transformation_guess.rotation = lidar_synced_state.ori;
    preint_transformation_guess.matrix = Eigen::Matrix4d::Identity();
    preint_transformation_guess.matrix.block<3,3>(0,0) = Eigen::Matrix3d(lidar_synced_state.ori);
    preint_transformation_guess.matrix.block<3,1>(0,3) = Eigen::Vector3d(lidar_synced_state.pos);

}

INSstate INS::calcKinematicInterpolatedState(const INSstate & this_state, const INSstate & next_state, double interpolate_time)
{
    double sub_calc_time{};
    // calculates the imu interpolated between 2 given states with and a sync time
    double sync_time = interpolate_time;
    double half_sync_time_sq = 1.0/2.0 * sync_time*sync_time;
    double sixth_sync_time_cb = 1.0/3.0 * half_sync_time_sq*sync_time;
    double imu_dt = this_state.dt;

    INSstate interpolated_state;
    interpolated_state.ori = this_state.ori;


    // interpolated_state.pos = this_state.pos + this_state.vel * sync_time + (preint_states[0].ori *preint_states[0].acc) * half_sync_time_sq  +  preint_states[0].jerk * sixth_sync_time_cb;
    // interpolated_state.vel = this_state.vel + (preint_states[0].ori.matrix() *preint_states[0].acc) *sync_time + preint_states[0].jerk * half_sync_time_sq;
    // interpolated_state.acc = this_state.acc + preint_states[0].ori.inverse().matrix() * preint_states[0].jerk * sync_time;
    // interpolated_state.ang = this_state.ang + preint_states[0].alpha * sync_time;

    // interpolated_state.pos = this_state.pos + this_state.vel * sync_time + (this_state.ori * this_state.acc) * half_sync_time_sq  + this_state.jerk * sixth_sync_time_cb;
    // interpolated_state.vel = this_state.vel + (this_state.ori * this_state.acc) *sync_time + this_state.jerk * half_sync_time_sq;
    // interpolated_state.acc = this_state.acc + this_state.ori.inverse() * this_state.jerk * sync_time;
    // interpolated_state.ang = this_state.ang + this_state.alpha * sync_time;


    auto t1 = std::chrono::high_resolution_clock::now();
    Eigen::Matrix3d ori_matrix = this_state.ori.toRotationMatrix();
    interpolated_state.pos = this_state.pos + this_state.vel * sync_time + (ori_matrix * this_state.acc) * half_sync_time_sq  + this_state.jerk * sixth_sync_time_cb;
    interpolated_state.vel = this_state.vel + (ori_matrix * this_state.acc) *sync_time + this_state.jerk * half_sync_time_sq;
    interpolated_state.acc = this_state.acc + ori_matrix.inverse() * this_state.jerk * sync_time;
    interpolated_state.ang = this_state.ang + this_state.alpha * sync_time;


    // Eigen::Matrix3d ori_matrix = deltaQ(this_state.alpha * half_sync_time_sq).matrix() * (deltaQ(this_state.ang * sync_time) * this_state.ori).matrix();
    // interpolated_state.ori =  (Eigen::Quaterniond(ori_matrix));
    interpolated_state.ori =  (deltaQ(this_state.alpha * half_sync_time_sq) * (deltaQ(this_state.ang * sync_time) * this_state.ori)).normalized();
    // updateOrientationFromAngularRate(interpolated_state.ori, this_state.ang, sync_time);
    // updateOrientationFromAngularRate(interpolated_state.ori, this_state.alpha, half_sync_time_sq);

    // Eigen::Quaterniond dq_vel = multQuatByScalar(deltaQ(this_state.ang), sync_time);
    // (deltaQ(this_state.alpha * half_sync_time_sq) *
    // interpolated_state.ori =   addQuaternions(this_state.ori * dq_vel, this_state.ori).normalized();
    // interpolated_state.ori =   addQuaternions(this_state.ori, this_state.ori).normalized();
    
    // linear interpolated jerk and alpha, interpolated from the sync_time to imu_dt
    double interpolate_ratio = sync_time / imu_dt; // sync_time should be between 0 and imu_dt, if not something is wrong
    Eigen::Vector3d jerk  = this_state.jerk * (1.0 - interpolate_ratio)   +  next_state.jerk * interpolate_ratio; 
    Eigen::Vector3d alpha = this_state.alpha * (1.0 - interpolate_ratio)  +  next_state.alpha * interpolate_ratio; 
    interpolated_state.jerk = jerk;
    interpolated_state.alpha = alpha;
    
    interpolated_state.time =  this_state.time + sync_time;
    interpolated_state.dt =  imu_dt; // not sure this should be the full dt, maybe imu_dt - sync_time?
    auto t2 = std::chrono::high_resolution_clock::now();

    sub_calc_time += getTimeDouble(t2 - t1) *1000;

    // cout << "interpolating time: " << sub_calc_time << "ms \n";

    return interpolated_state;
}

void INS::undistortCloud(const pcl::PointCloud<PointType> &cloud_in, pcl::PointCloud<PointType> &cloud_out)
{   
    // create state sync look-up index
    size_t cloud_size = cloud_in.points.size();

    int *state_sync_index = new int[cloud_size];
    double *point_frame_dts = new double[cloud_size];
    #pragma omp parallel for shared(state_sync_index, point_frame_dts)
    for (size_t k = 0; k < cloud_size; k++){
        int i = 0;
        PointType point = cloud_in.points[k];
        double intensity = point.intensity;
        double point_frame_dt = intensity - (int)intensity; // point timestamps are saved as the decimal part of the intensity values that are ints
        while ((point_frame_dt + lidar_synced_state.time) >= preint_states[i+1].time){
            i++;
            // RCLCPP_INFO(get_logger(),"state sync index %i to point index k %i", i, k); 
        }
        state_sync_index[k] = i;
        point_frame_dts[k] = point_frame_dt ;
    }


    auto u1 = std::chrono::high_resolution_clock::now();
    double sub_calc_time{};
    double full_calc_time{};
    // INSstate stateX;
    INSstate stateX;
    INSstate state_next;
    // Eigen::Matrix4d T_star = Eigen::Matrix4d::Identity();
    // double point_state_dt;
    #pragma omp parallel for reduction(+:sub_calc_time) reduction(+:full_calc_time) private(stateX, state_next) shared(state_sync_index, point_frame_dts) num_threads(8)
    // #pragma omp parallel for private(stateX, state_next) shared(state_sync_index, point_frame_dts) num_threads(8)
    for (size_t k = 0; k < cloud_size; k++){
        auto v1 = std::chrono::high_resolution_clock::now();
        // rclcpp::Time time_calc_start = system_clock.now();
        PointType point = cloud_in.points[k];
        int i = state_sync_index[k];
        double point_frame_dt = point_frame_dts[k];
        double sync_time = 0.0; // sync_time should be negative, time between scan start and preceding imu measurement
        if (i == 0) {
            // RCLCPP_INFO(get_logger(), "i == 0, k = %i", k);
            stateX = lidar_synced_state;
            state_next = preint_states[1];
        } else {
            stateX = preint_states[i];
            state_next = preint_states[i+1]; // WHAT IF ARRAY IS ONLY 'i' BIG?! -> apparantly this is not a problem
            sync_time = lidar_synced_state.time - stateX.time ;
        }
        double point_state_dt = point_frame_dt + sync_time; 

        if (point_state_dt < 0.0){  // error detection.. only really happens if the dt is "negative zero" - beyond machine precision
            // RCLCPP_WARN(get_logger(), "Negative point dt detected state dt %f, frame dt %f, sync time %f, index %i", point_state_dt, point_frame_dt, sync_time, k);
            // cout << "Negative point dt detected state dt "<< to_string(point_state_dt) <<", frame dt  "<< to_string(point_frame_dt) <<", sync time  "<< to_string(sync_time) <<", index  "<< to_string(k) << "\n";
            point_state_dt = +0.0;
        }

        if (!pcl::isFinite(point))
        {
            // RCLCPP_INFO_ONCE(get_logger(), "point NAN! before undistortion");
            cout << "point NAN! before undistortion" << "\n";
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        INSstate point_undistort_state = calcKinematicInterpolatedState(stateX, state_next, point_state_dt); // returns the interpolated kinematic state at the point time
        
        Eigen::Vector3d point_vector(point.x, point.y, point.z); 
        // Eigen::Vector4d point_vector(point.x, point.y, point.z, 1.0); // homogenous representatation of point
        Eigen::Vector3d point_normal_vector(point.normal_x, point.normal_y, point.normal_z);

        Eigen::Vector3d point_translation_distortion = point_undistort_state.pos;
        Eigen::Quaterniond point_rotation_distortion = point_undistort_state.ori;

        // apply transformation to point
        // Eigen::Vector3d point_vector(point.x, point.y, point.z);
        point_vector = point_rotation_distortion * point_vector + point_translation_distortion;

        // apply transformation to point
        // T_star.block<3,3>(0,0) = point_rotation_distortion.matrix();
        // T_star.block<3,1>(0,3) = point_translation_distortion;
        // point_vector = T_star*point_vector;
        // point_normal_vector = T_star.block<3,3>(0,0)*point_normal_vector;

        point.x = point_vector.x();
        point.y = point_vector.y();
        point.z = point_vector.z();

        point.normal_x = point_normal_vector.x();
        point.normal_y = point_normal_vector.y();
        point.normal_z = point_normal_vector.z();

        cloud_out.points[k] = point;
        auto t2 = std::chrono::high_resolution_clock::now();

        // if (!pcl::isFinite(*point))
        if (!pcl::isFinite(point))
        {
            cout << "point NAN After undistortion!" << "\n";
        }

        // rclcpp::Time time_calc_end = system_clock.now();
        // sub_calc_time += (time_calc_end.seconds() - time_calc_start.seconds()) * 1000.0;
        
        auto v2 = std::chrono::high_resolution_clock::now();
        sub_calc_time += getTimeDouble(t2 - t1) *1000;
        full_calc_time += getTimeDouble(v2 - v1) *1000;
    }
    auto u2 = std::chrono::high_resolution_clock::now();
    double undistort_time = getTimeDouble(u2 - u1) *1000;

    // rclcpp::Time time_undistort_end = system_clock.now();
    // undistort_time = (time_undistort_end.seconds() - time_undistort_start.seconds()) * 1000.0;

    delete[] state_sync_index; // clear the memory usage
    delete[] point_frame_dts;

    // RCLCPP_INFO(get_logger(), "---- UNDISTORTION ---- ");
    // RCLCPP_INFO(get_logger(), "Undistortion of %i points, MP cpu time: %fms, avg sub-calc time: %fms", cloud_in.points.size(), undistort_time,  sub_calc_time/(float)cloud_in.points.size() );
    // RCLCPP_INFO(get_logger(), "Total non-MP process time: %fms", sub_calc_time );
    cout << "Undistortion of " << cloud_in.points.size()<< " points\nMP cpu time: " << undistort_time  << " ms\navg sub-calc time: " <<  sub_calc_time/(float)cloud_in.points.size() << " ms\ntotal sub calc time: "<< sub_calc_time <<" ms\ntotal non-MP time: "<< full_calc_time <<" ms\n";
}

const INSstate& INS::getPreintState(int index)
{
    return this->preint_states[index];
}

const INSstate& INS::getPredictState(int index)
{
    return this->predict_states[index];
}

// Eigen::Quaterniond getMadgwickOrientation(double timestamp)
// {
// }

///////////////////////////////////////////////////////////////////////////////////////////

// /** \brief This is a mock class with the sole purpose of accessing a protected member of a class it inherits from.
// *
// * Some of the relevant documentation for correspondences: http://docs.pointclouds.org/trunk/correspondence_8h_source.html#l00092
// */
// template <typename PointSource, typename PointTarget, typename Scalar = float>
// class IterativeClosestPointNonLinear_Exposed : public pcl::IterativeClosestPointNonLinear<PointSource, PointTarget, Scalar> {
//   public:
//     pcl::CorrespondencesPtr getCorrespondencesPtr() {
//       for (uint32_t i = 0; i < this->correspondences_->size(); i++) {
//         pcl::Correspondence currentCorrespondence = (*this->correspondences_)[i];
//         std::cout << "Index of the source point: " << currentCorrespondence.index_query << std::endl;
//         std::cout << "Index of the matching target point: " << currentCorrespondence.index_match << std::endl;
//         std::cout << "Distance between the corresponding points: " << currentCorrespondence.distance << std::endl;
//         std::cout << "Weight of the confidence in the correspondence: " << currentCorrespondence.weight << std::endl;
//       }
//       return this->correspondences_;
//     }
// };



class LidarOdometry : public rclcpp::Node 
{
    private:
        rclcpp::Clock run_clock;

        //callback groups
        rclcpp::CallbackGroup::SharedPtr subscriber_cb_group_;
        rclcpp::CallbackGroup::SharedPtr subscriber_cb_group2_;
        rclcpp::CallbackGroup::SharedPtr run_cb_group_;

        rclcpp::TimerBase::SharedPtr run_timer;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_data_service;

        // pose representation: [quaternion: w, x, y, z | transition: x, y, z]
        // double abs_pose[7];   //absolute pose from current frame to the first frame / odometry
        // double rel_pose[7];   //relative pose between two frames

        
        boost::shared_ptr<pcl::Registration < PointType, PointType >> registration_;
        boost::shared_ptr<pcl::Registration < PointType, PointType >> registration_fuse_;

        rclcpp::Clock system_clock;
        double registration_process_time;

        double system_start_time{};
        double current_subframe_time{};

        int full_frame_count{};
        int sub_frame_count{};


        bool system_initialized = false;
        bool new_cloud_ready = false;
        bool init_map_built = false;

        size_t latest_frame_idx;
        size_t latest_keyframe_idx;

        double icp_fitness = 0.0;
        deque<double> fitnesses;

        deque<double> imu_dts;
        double last_imu_time{};

        double first_lidar_msg_time{};
        double scan_dt{};
        double current_scan_time{};
        // double next_scan_time{};

        double cloud_scale;
        double prev_cloud_scale;

        double crop_pile_end_time;

        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg;

        Eigen::Matrix4d initial_pose_matrix;
        Pose initial_pose;

        // transformation matrices
        Transformation registration_transformation;
        Transformation last_odometry_transformation;
        Transformation preint_transformation_guess;
        Transformation keyframe_to_odometry_transformation;
        Transformation preint_residual;
        Pose last_odometry_pose;

        Pose ins_pose_new;
        Pose ins_pose;
        Transformation ins_relative;

        // Eigen::Matrix4d registration_transformation;
        // Eigen::Matrix4d last_odometry_transformation;
        // Eigen::Matrix4d last_odometry_pose;
        Eigen::Matrix4d odometry_transformation_guess;

        Eigen::Matrix4f init_guess;


        // deque<Eigen::Matrix4d> keyframe_poses; // keyframes class instead?
        Pose keyframe_pose;
        deque<Pose> keyframe_poses; // keyframes class instead?
        // deque<size_t> keyframe_index; // keyframes class instead?



        // headers and header information
        rclcpp::Time time_new_cloud;
        std_msgs::msg::Header cloud_header;

        nav_msgs::msg::Odometry odom;
        nav_msgs::msg::Path path;
        nav_msgs::msg::Path path_ins;
        nav_msgs::msg::Path path_predicted_ins;

        geometry_msgs::msg::PoseWithCovarianceStamped transformation_geommsg;

        // pcl filters downsampling
        pcl::VoxelGrid<PointType> down_size_filter;
        pcl::VoxelGrid<PointType> down_size_filter_local_map;
        pcl::VoxelGrid<PointType> down_size_filter_keyframe_map;

        //point clouds and vector containing pointclouds
        pcl::PointCloud<PointType>::Ptr cloud_in = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr cloud_in_ds = boost::make_shared<pcl::PointCloud<PointType>>();
        // pcl::PointCloud<PointType>::Ptr cloud_prev = boost::make_shared<pcl::PointCloud<PointType>>();
        // pcl::PointCloud<PointType>::Ptr cloud_prev_ds = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr cloud_keyframe = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr cloud_keyframe_ds = boost::make_shared<pcl::PointCloud<PointType>>();

        pcl::PointCloud<PointType>::Ptr crop_pile = boost::make_shared<pcl::PointCloud<PointType>>();
        // pcl::PointCloud<PointType>::Ptr subframe = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr current_subframe = boost::make_shared<pcl::PointCloud<PointType>>();
        deque<pcl::PointCloud<PointType>::Ptr> subframes;

        deque<double> crop_point_stamps;


        deque<sensor_msgs::msg::PointCloud2> cloud_queue;
        sensor_msgs::msg::PointCloud2 current_cloud_msg;

        pcl::PointCloud<PointType>::Ptr keyframe_map = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr keyframe_map_ds = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr local_map = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr local_map_ds = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr long_term_map = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr target_map = boost::make_shared<pcl::PointCloud<PointType>>();

        pcl::PointCloud<PointType>::Ptr reduced_global_map = boost::make_shared<pcl::PointCloud<PointType>>();

        deque<pcl::PointCloud<PointType>::Ptr> recent_frames;


        vector<boost::shared_ptr<pcl::PointCloud<PointType>>> all_clouds;

        // "point cloud" containing a specific type that has the odometry information
        pcl::PointCloud<PoseInfo>::Ptr odometry_pose_info = boost::make_shared<pcl::PointCloud<PoseInfo>>(); // add keyframe pose info?
        pcl::PointCloud<PoseInfo>::Ptr keyframe_pose_info = boost::make_shared<pcl::PointCloud<PoseInfo>>(); // add keyframe pose info?
        // pcl::PointCloud<pcl::PointXYZI>::Ptr odometry_pose_positions = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr long_term_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub;
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ins_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr bias_sub;

        //publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fullpointcloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloudprior_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_cloud_pub;
        // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalcloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localcloud_pub;

        // rclcpp::Publisher<std_msgs::msg::Int64 >::SharedPtr keyframe_idx_pub;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ins_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr keyframe_odometry_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_ins_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_predicted_ins_pub;

        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr lidar_odometry_transformation_pub;

        deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer;

        deque<std::pair<sensor_msgs::msg::Imu::SharedPtr, pcl::PointCloud<PointType>::Ptr>> synchronized_imu_subframes_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        double translation_std_x;
        double translation_std_y;
        double translation_std_z;

        double translation_std_min_x_;
        double translation_std_min_y_;
        double translation_std_min_z_;


        // Eigen::Quaterniond preint_quat ;
        // Eigen::Vector3d preint_position;
        // Eigen::Vector3d preint_velocity;

        INS ins; // the new ins class that will handle INS and preintegration
        // rclcpp::Node ins; // the new ins class that will handle INS and preintegration

        sensor_msgs::msg::Imu::SharedPtr current_imu_msg;

        INSstate state0;

        INSstate preint_state;
        INSstate observer_state;
        INSstate preint_anchor;
        IMUwrench preint_bias;
        // IMUwrench observer_bias;
        // IMUwrench kalman_bias;
        deque<INSstate> preint_states;


        Eigen::Vector3d average_translation;
        deque<Eigen::Vector3d> recent_translations;

        bool initial_pose_recieved{};
        bool first_lidar_received{};

        // parameters
        // ds_voxelsize
        // ds_voxelsize_lc
        float ds_voxel_size_;
        float ds_voxel_size_lc_;
        float ds_voxel_size_kf_;
        // strings
        std::string frame_id = "lidar_odom";
        // this->declare_parameter("my_parameter", "world");
        double keyframe_threshold_angle_;
        double keyframe_threshold_length_;
        double keyframe_threshold_fitness_;
        int keyframe_threshold_index_; // max frames between keyframes
        int icp_max_iterations_;
        int icp_max_coarse_iterations_;
        double icp_max_correspondence_distance_;
        int coarse_correspondence_factor_;
        int local_map_width_;
        int start_idx_;
        int end_idx_;

        size_t max_frames;


        int points_per_cloud_scale_;
        int scan_matching_method_;

        int local_map_init_frames_count_;

        bool use_cloud_scale_for_ds_{};
        bool use_wheel_constraint_{};
        bool use_ins_guess_{};
        bool use_preint_imu_guess_{};
        bool use_lidar_odometry_guess_{};
        bool use_preint_undistortion_{};

        bool save_running_data_{};

        double ds_lc_voxel_size_ratio_;
        double cloud_scale_previuos_cloud_weight_;

        double start_delay_s_;
        double sync_offset_ms_;


        double derotate_angle_x_;
        double derotate_angle_y_;
        double derotate_angle_z_;


        double gamma_1_;
        double gamma_2_;
        double gamma_3_;
        double gamma_4_;
        double gamma_5_;


        std::string imu_topic_;

        std::string save_path_;

        // publish topics as parameters so they can be changed?

        std::ofstream data_file;
    

    public:
        LidarOdometry()
        : Node("lidar_odometry")
        {

            // declare_parameter("start_delay_s", 0.0f);
            // get_parameter("start_delay_s", start_delay_s_);

            declare_parameter("sync_offset_ms", 0.1f);
            get_parameter("sync_offset_ms", sync_offset_ms_);

            // declare_parameter("ds_voxel_size", 0.1f);
            // get_parameter("ds_voxel_size", ds_voxel_size_);

            // declare_parameter("ds_voxel_size_lc", 0.1f);
            // get_parameter("ds_voxel_size_lc", ds_voxel_size_lc_);

            // declare_parameter("ds_voxel_size_kf", 0.5f);
            // get_parameter("ds_voxel_size_kf", ds_voxel_size_kf_);

            // declare_parameter("keyframe_threshold_length", 0.3);
            // get_parameter("keyframe_threshold_length", keyframe_threshold_length_);

            // declare_parameter("keyframe_threshold_angle", 2.0);
            // get_parameter("keyframe_threshold_angle", keyframe_threshold_angle_);

            // declare_parameter("keyframe_threshold_fitness", 0.1); // 0.5 * icp_correspondance threshold
            // get_parameter("keyframe_threshold_fitness", keyframe_threshold_fitness_);

            // declare_parameter("keyframe_threshold_index", 0); 
            // get_parameter("keyframe_threshold_index", keyframe_threshold_index_);

            declare_parameter("scan_matching_method", 0); 
            get_parameter("scan_matching_method", scan_matching_method_);

            declare_parameter("icp_max_iterations", 20); 
            get_parameter("icp_max_iterations", icp_max_iterations_);

            // declare_parameter("icp_max_coarse_iterations", 5); 
            // get_parameter("icp_max_coarse_iterations", icp_max_coarse_iterations_);

            declare_parameter("icp_max_correspondence_distance", 0.2); 
            get_parameter("icp_max_correspondence_distance", icp_max_correspondence_distance_);

            // declare_parameter("coarse_correspondence_factor", 10); 
            // get_parameter("coarse_correspondence_factor", coarse_correspondence_factor_);

            // declare_parameter("local_map_width", 20); 
            // get_parameter("local_map_width", local_map_width_);

            // declare_parameter("local_map_init_frames_count", 0); 
            // get_parameter("local_map_init_frames_count", local_map_init_frames_count_);

            // declare_parameter("use_cloud_scale_for_ds", false); 
            // get_parameter("use_cloud_scale_for_ds", use_cloud_scale_for_ds_);

            // declare_parameter("points_per_cloud_scale", 25); 
            // get_parameter("points_per_cloud_scale", points_per_cloud_scale_);

            // declare_parameter("imu_topic", "/livox/imu"); 
            declare_parameter("imu_topic", "/imu/data_raw"); 
            get_parameter("imu_topic", imu_topic_);

            // declare_parameter("use_ins_guess", false); 
            // get_parameter("use_ins_guess", use_ins_guess_);

            // declare_parameter("use_preint_imu_guess", false); 
            // get_parameter("use_preint_imu_guess", use_preint_imu_guess_);

            // declare_parameter("use_lidar_odometry_guess", true); 
            // get_parameter("use_lidar_odometry_guess", use_lidar_odometry_guess_);

            // declare_parameter("use_preint_undistortion", true); 
            // get_parameter("use_preint_undistortion", use_preint_undistortion_);

            // declare_parameter("cloud_scale_previuos_cloud_weight", 0.5); 
            // get_parameter("cloud_scale_previuos_cloud_weight", cloud_scale_previuos_cloud_weight_);

            // declare_parameter("ds_lc_voxel_size_ratio", 3.0); 
            // get_parameter("ds_lc_voxel_size_ratio", ds_lc_voxel_size_ratio_);

            // declare_parameter("derotate_angle_x", 0.0); 
            // get_parameter("derotate_angle_x", derotate_angle_x_);
            // declare_parameter("derotate_angle_y", 0.0); 
            // get_parameter("derotate_angle_y", derotate_angle_y_);
            // declare_parameter("derotate_angle_z", 0.0); 
            // get_parameter("derotate_angle_z", derotate_angle_z_);

            // declare_parameter("translation_std_min_x", 0.01); 
            // get_parameter("translation_std_min_x", translation_std_min_x_);
            // declare_parameter("translation_std_min_y", 0.01); 
            // get_parameter("translation_std_min_y", translation_std_min_y_);
            // declare_parameter("translation_std_min_z", 0.01); 
            // get_parameter("translation_std_min_z", translation_std_min_z_);

            // declare_parameter("gamma_1", 10.0); 
            // get_parameter("gamma_1", gamma_1_);
            // declare_parameter("gamma_2", 10.0); 
            // get_parameter("gamma_2", gamma_2_);
            // declare_parameter("gamma_3", 10.0); 
            // get_parameter("gamma_3", gamma_3_);
            // declare_parameter("gamma_4", 10.0); 
            // get_parameter("gamma_4", gamma_4_);
            // declare_parameter("gamma_5", 10.0); 
            // get_parameter("gamma_5", gamma_5_);

            declare_parameter("save_running_data", true); 
            get_parameter("save_running_data", save_running_data_);
            
            declare_parameter("save_path", "temp_saved_odometry_data/run_data.csv"); 
            get_parameter("save_path", save_path_);


            // declare_parameter("start_idx", 0); 
            // get_parameter("start_idx", start_idx_);

            // declare_parameter("end_idx", std::numeric_limits<int>::max ()); 
            // declare_parameter("end_idx", 0); // 0 means endless 
            // get_parameter("end_idx", end_idx_);

            // RCLCPP_INFO(get_logger(), "ds_voxel_size in constructor is: %f", ds_voxel_size_);

            printDeclaredParameters();


            initializeParameters();
            initializeScanmatcher();
            allocateMemory();

            // setup callback groups
            run_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            // publsiher callback groud added?
            subscriber_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // create subscriber callback group
            rclcpp::SubscriptionOptions options; // create subscribver options
            options.callback_group = subscriber_cb_group_; // add callbackgroup to subscriber options

            subscriber_cb_group2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // create subscriber callback group
            rclcpp::SubscriptionOptions options2; // create subscribver options
            options2.callback_group = subscriber_cb_group2_; // add callbackgroup to subscriber options

            run_timer = this->create_wall_timer(1ms, std::bind(&LidarOdometry::run, this), run_cb_group_); // the process timer 

            // save_data_service = this->create_service<std_srvs::srv::Trigger>("save_odometry_data", &LidarOdometry::save_data, rmw_qos_profile_services_default , subscriber_cb_group_);
            save_data_service = this->create_service<std_srvs::srv::Trigger>("save_odometry_data", std::bind(&LidarOdometry::saveDataService,this, std::placeholders::_1, std::placeholders::_2));


            pointcloud_sub_ = this->create_subscription<PC_msg>("/preprocessed_point_cloud", 100, std::bind(&LidarOdometry::pointCloudHandler, this, _1), options);

            // pointcloud_sub_ = this->create_subscription<PC_msg>("/surf_features", 100, std::bind(&LidarOdometry::pointCloudHandler, this, _1), options);
            // pointcloud_sub_ = this->create_subscription<PC_msg>("/edge_features", 100, std::bind(&LidarOdometry::pointCloudHandler, this, _1), options);
            // initial_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initial_pose", 100, std::bind(&LidarOdometry::initialPoseHandler, this, _1), options);
            // ins_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom_ins", 100, std::bind(&LidarOdometry::insHandler, this, _1), options);
            imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, 1000, std::bind(&LidarOdometry::imuDataHandler, this, _1), options2);

            pointcloud_pub = this->create_publisher<PC_msg>("/ds_point_cloud_odom", 10);
            fullpointcloud_pub = this->create_publisher<PC_msg>("/full_point_cloud_odom", 10);
            pointcloudprior_pub = this->create_publisher<PC_msg>("/full_point_cloud_transform_guess", 10);
            // keyframe_cloud_pub = this->create_publisher<PC_msg>("/full_point_cloud_keyframe", 2);

            // globalcloud_pub = this->create_publisher<PC_msg>("/global_point_cloud", 100);
            localcloud_pub = this->create_publisher<PC_msg>("/local_point_cloud", 10);

            // current odometry publisher
            odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 25);
            // keyframe_odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom_keyframe", 25);
            // ins_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom_ins", 10);

            path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
            // path_ins_pub = this->create_publisher<nav_msgs::msg::Path>("/path_ins", 10);
            // path_predicted_ins_pub = this->create_publisher<nav_msgs::msg::Path>("/path_predicted_ins", 10);

            // lidar_odometry_transformation_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/transformation/lidar", 100);

            // keyframe_idx_pub = this->create_publisher<std_msgs::msg::Int64>("keyframe_idx", 100);

            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  
            initial_pose_matrix = Eigen::Matrix4d::Identity();
            initial_pose.orientation = Eigen::Quaterniond::Identity();
            initial_pose.position = Eigen::Vector3d::Zero();

            preint_state.ori = Eigen::Quaterniond::Identity();
            preint_state.pos = Eigen::Vector3d::Zero();
            preint_state.vel = Eigen::Vector3d::Zero();
            preint_state.acc = Eigen::Vector3d::Zero();
            preint_state.time = 0.0;

            preint_bias.acc = Eigen::Vector3d::Zero();
            preint_bias.ang = Eigen::Vector3d::Zero();
            preint_bias.time = 0.0;
            preint_state.bias = preint_bias;

            observer_state = preint_state;

            // observer_bias = preint_bias;
            // kalman_bias = preint_bias;

            ins = INS(); // maybe add an expected dt from rosparameter input 
        
            initDataFile();
        }
        ~LidarOdometry(){}

        void saveDataService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response>  response){
        // void save_data(){
            RCLCPP_INFO(get_logger(), "Request to save data received %d.", request);
            bool data_to_be_saved = fitnesses.size() > 0;
            size_t elements_to_save = fitnesses.size();

            if (data_to_be_saved){

                std::ofstream file;
                file.open ("temp_saved_odometry_data/fitness.csv");
                for(size_t i=0; i < elements_to_save; i++){
                    double fitn = fitnesses.front();
                    file << std::to_string(fitn) + "\n"; 
                    fitnesses.pop_front();
                }

                // file << "This is the first cell in the first column.\n";
                // file << "a,b,c,\n";
                // file << "c,s,v,\n";
                // file << "1,2,3.456\n";
                // file << "semi;colon";
                file.close();
            } 

            bool data_emptied_to_file = fitnesses.size() == 0;

            response->success = data_to_be_saved && data_emptied_to_file;
            response->message = "End of service" ;

            RCLCPP_INFO(get_logger(), "Data has been saved, Data: %s, Emptied: %s ", data_to_be_saved? "true":"false", data_emptied_to_file?"true":"false");
        
        }

        void initDataFile()
        {   
            if (!save_running_data_)
                return;

            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);
            std::ostringstream oss;
            oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
            std::string datestr = oss.str();


            data_file.open (save_path_ + datestr + "_run_data.csv");

            // write the header to the save file

            data_file << "ds_voxel_size: " + std::to_string(ds_voxel_size_)+"\n";
            data_file << "ds_voxel_size_lc: "+ std::to_string(ds_voxel_size_lc_)+"\n";
            data_file << "use_cloud_scale_for_ds: "+ std::to_string(use_cloud_scale_for_ds_)+"\n";
            data_file << "cloud_scale_previuos_cloud_weight: "+ std::to_string(cloud_scale_previuos_cloud_weight_)+"\n";
            data_file << "ds_lc_voxel_size_ratio: "+ std::to_string(ds_lc_voxel_size_ratio_)+"\n";
            data_file << "points_per_cloud_scale: "+ std::to_string(points_per_cloud_scale_)+"\n";
            data_file << "keyframe_threshold_length: "+ std::to_string(keyframe_threshold_length_)+"\n";
            data_file << "keyframe_threshold_angle: "+ std::to_string(keyframe_threshold_angle_)+"\n";
            data_file << "keyframe_threshold_fitness: "+ std::to_string(keyframe_threshold_fitness_)+"\n";
            data_file << "keyframe_threshold_index: "+ std::to_string(keyframe_threshold_index_)+"\n";
            data_file << "scan_matching_method: "+ std::to_string(scan_matching_method_)+"\n";
            data_file << "icp_max_iterations: "+ std::to_string(icp_max_iterations_)+"\n";
            data_file << "icp_max_coarse_iterations: "+ std::to_string(icp_max_coarse_iterations_)+"\n";
            data_file << "icp_max_correspondence_distance: "+ std::to_string(icp_max_correspondence_distance_)+"\n";
            data_file << "coarse_correspondence_factor: "+ std::to_string(coarse_correspondence_factor_)+"\n";
            data_file << "local_map_width: "+ std::to_string(local_map_width_)+"\n";
            data_file << "local_map_init_frames_count: "+ std::to_string(local_map_init_frames_count_)+"\n";
            data_file << "imu_topic:"+ imu_topic_+"\n";
            data_file << "use_ins_guess: "+ std::to_string(use_ins_guess_)+"\n";
            data_file << "use_preint_imu_guess: "+ std::to_string(use_preint_imu_guess_)+"\n";
            data_file << "use_lidar_odometry_guess: "+ std::to_string(use_lidar_odometry_guess_)+"\n";
            data_file << "use_preint_undistortion: "+ std::to_string(use_preint_undistortion_)+"\n";
            data_file << "sync_offset_ms: "+ std::to_string(sync_offset_ms_)+"\n";
            data_file << "translation_std_min_x: "+ std::to_string(translation_std_min_x_)+"\n";
            data_file << "translation_std_min_y: "+ std::to_string(translation_std_min_y_)+"\n";
            data_file << "translation_std_min_z: "+ std::to_string(translation_std_min_z_)+"\n";
            data_file << "gamma_1: "+ std::to_string(gamma_1_)+"\n";
            data_file << "gamma_2: "+ std::to_string(gamma_2_)+"\n";
            data_file << "gamma_3: "+ std::to_string(gamma_3_)+"\n";
            data_file << "gamma_4: "+ std::to_string(gamma_4_)+"\n";
            data_file << "gamma_5: "+ std::to_string(gamma_5_)+"\n";

            data_file << "time, x, y, z, vx, vy, vz, obs_vx, obs_vy, obs_vz, qw, qx, qy, qz, fitness, residual_x, residual_y, residual_z, residual_qw, residual_qx, residual_qy, residual_qz, cov_x, cov_y, cov_z, bias_acc_x, bias_acc_y, bias_acc_z, bias_ang_x, bias_ang_y, bias_ang_z";

        }

        void saveRunningData()
        {
            if (!save_running_data_)
                return;

            RCLCPP_DEBUG(get_logger(), "---- SAVING DATA ----");
            rclcpp::Time time_save_start = system_clock.now();
            // order of data: "time, x, y, z, vx, vy, vz, obs_vx, obs_vy, obs_vz, qw, qx, qy, qz, fitness, residual_x, residual_y, residual_z, residual_qw, residual_qx, residual_qy, residual_qz, cov_x, cov_y, cov_z, bias_acc_x, bias_acc_y, bias_acc_z, bias_ang_x, bias_ang_y, bias_ang_z "
            deque<double> print_qeue;
            print_qeue.push_back(current_scan_time);                    // time
            print_qeue.push_back(last_odometry_pose.position.x());      // x          
            print_qeue.push_back(last_odometry_pose.position.y());      // y          
            print_qeue.push_back(last_odometry_pose.position.z());      // z  
            print_qeue.push_back(last_odometry_pose.velocity.x());      // vx          
            print_qeue.push_back(last_odometry_pose.velocity.y());      // vy          
            print_qeue.push_back(last_odometry_pose.velocity.z());      // vz  
            print_qeue.push_back(observer_state.vel.x());               // obs_vx          
            print_qeue.push_back(observer_state.vel.y());               // obs_vy          
            print_qeue.push_back(observer_state.vel.z());               // obs_vz          
            print_qeue.push_back(last_odometry_pose.orientation.w());   // qw         
            print_qeue.push_back(last_odometry_pose.orientation.x());   // qx         
            print_qeue.push_back(last_odometry_pose.orientation.y());   // qy         
            print_qeue.push_back(last_odometry_pose.orientation.z());   // qz         
            print_qeue.push_back(icp_fitness);                          // fitness 
            print_qeue.push_back(preint_residual.translation.x());      // residual_x
            print_qeue.push_back(preint_residual.translation.y());      // residual_y
            print_qeue.push_back(preint_residual.translation.z());      // residual_z
            print_qeue.push_back(preint_residual.rotation.w());         // residual_qw 
            print_qeue.push_back(preint_residual.rotation.x());         // residual_qx 
            print_qeue.push_back(preint_residual.rotation.y());         // residual_qy 
            print_qeue.push_back(preint_residual.rotation.z());         // residual_qz 
            print_qeue.push_back(translation_std_x*translation_std_x);  // cov_x 
            print_qeue.push_back(translation_std_y*translation_std_y);  // cov_y 
            print_qeue.push_back(translation_std_z*translation_std_z);  // cov_z 
            print_qeue.push_back(observer_state.bias.acc.x());          // bias_acc_x          
            print_qeue.push_back(observer_state.bias.acc.y());          // bias_acc_y         
            print_qeue.push_back(observer_state.bias.acc.z());          // bias_acc_z          
            print_qeue.push_back(observer_state.bias.ang.x());          // bias_ang_x          
            print_qeue.push_back(observer_state.bias.ang.y());          // bias_ang_y         
            print_qeue.push_back(observer_state.bias.ang.z());          // bias_ang_z          

            int length =  (int)print_qeue.size();

            data_file << "\n";
            for(int i=0; i < length; i++){
                data_file << std::to_string(print_qeue.front()); 
                if (i < length-1){
                    data_file << ","; 
                }
                print_qeue.pop_front();
            }
            print_qeue.clear(); // just to make sure it is empty

            rclcpp::Time time_save_end = system_clock.now();

            RCLCPP_DEBUG(get_logger(), "Odometry data saved: %fms", time_save_end.seconds()*1000.0 - time_save_start.seconds()*1000.0);
        }

        void printDeclaredParameters(){
            // Retrieve all declared parameters
            auto parameters = this->list_parameters({}, 10);  // List parameters with a maximum depth of 10

            RCLCPP_INFO(get_logger(),"Declared parameters: ");
            // Print parameter names and values
            for (const auto & param_name : parameters.names) {
                auto param_value = this->get_parameter(param_name);
                RCLCPP_INFO(get_logger(), "Parameter: %s : %s", param_name.c_str(), paramValueToString(param_value).c_str());
                // std::cout << "Parameter: " << param_name << " Value: " << paramValueToString(param_value) << std::endl;
            }
        }
        std::string paramValueToString(const rclcpp::Parameter & param)
        {
            // Convert parameter value to string based on its type
            switch (param.get_type()) {
                case rclcpp::ParameterType::PARAMETER_BOOL:
                    return param.as_bool() ? "true" : "false";
                case rclcpp::ParameterType::PARAMETER_INTEGER:
                    return std::to_string(param.as_int());
                case rclcpp::ParameterType::PARAMETER_DOUBLE:
                    return std::to_string(param.as_double());
                case rclcpp::ParameterType::PARAMETER_STRING:
                    return param.as_string();
                case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
                    return "byte array";  // Custom handling if needed
                case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
                    return "bool array";  // Custom handling if needed
                case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
                    return "integer array";  // Custom handling if needed
                case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
                    return "double array";  // Custom handling if needed
                case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
                    return "string array";  // Custom handling if needed
                case rclcpp::ParameterType::PARAMETER_NOT_SET:
                default:
                    return "unknown";
            }
        }

        void derotateCloud(const pcl::PointCloud<PointType> &cloud_in, pcl::PointCloud<PointType> &cloud_out){
            // Define the rotation matrix
            Eigen::AngleAxisf rotation_x((float)derotate_angle_x_/180.0 *M_PI, Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf rotation_y((float)derotate_angle_y_/180.0 *M_PI, Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf rotation_z((float)derotate_angle_z_/180.0 *M_PI, Eigen::Vector3f::UnitZ());
            Eigen::Matrix3f rotation_matrix = (rotation_z * rotation_y * rotation_x).toRotationMatrix();
            Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
            transformation_matrix.block<3,3>(0,0) = rotation_matrix;

            // Apply the rotation to the input cloud
            // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloudWithNormals<PointType>(cloud_in, cloud_out, transformation_matrix);
        }
    

        void pointCloudHandler(const PC_msg::SharedPtr lidar_cloud_msg )
        {   
            if (!first_lidar_received) {
                first_lidar_msg_time = toSec(lidar_cloud_msg->header.stamp);
            }

            RCLCPP_INFO_ONCE(get_logger(), "First Lidar cloud received - timestamp: %f..", first_lidar_msg_time);
            
            first_lidar_received = true;
            cloud_queue.push_back(*lidar_cloud_msg);
            // addFrameToCropPile();

        }
        
        void addFrameToCropPile()
        {
            if (cloud_queue.size() <= 0) {return; }

            current_cloud_msg = cloud_queue.front(); // puts the next cloud to variable current
            double scan_start_time = toSec(current_cloud_msg.header.stamp) + sync_offset_ms_ *1e-3;
            // check that imu msg covers the entire cloud
            fromROSMsg(current_cloud_msg, *cloud_in);
            
            // get individual point timestamps
            vector<double> point_timestamps;
            for (size_t i=0; i < cloud_in->points.size(); i++){
                double timestamp = cloud_in->points[i].intensity - (int)cloud_in->points[i].intensity + scan_start_time;
                point_timestamps.push_back(timestamp);
            }


            double scan_end_time = point_timestamps.back();
            // double scan_dt = scan_end_time - scan_start_time;

            // if (crop_pile_end_time < last_imu_time){

            crop_point_stamps.insert(crop_point_stamps.end(), point_timestamps.begin(), point_timestamps.end());
            *crop_pile += *cloud_in;

            crop_pile_end_time = scan_end_time;
            cloud_queue.pop_front(); // removes element from the queue
            full_frame_count++;
            RCLCPP_INFO(get_logger(), "Points added to crop pile, frame count %i", full_frame_count); 
            // }
    
        
        }

        void syncCropLidar()
        {
            // get the next cloud in the buffer, and check if there are imu msg to cover the entire cloud..
            // get next frame in buffer..

            if (imu_buffer.size() <= 1) { return; }
            double imu_end_time = toSec(imu_buffer[1]->header.stamp); 
   
            while (imu_end_time <= crop_point_stamps.front()){ // discard imu measurements before first lidar frame... 
                // RCLCPP_INFO(get_logger(), "IMU buffer size? %i", imu_buffer.size()); 
                imu_buffer.pop_front();
                imu_end_time = toSec(imu_buffer[1]->header.stamp); 
                // RCLCPP_INFO(get_logger(), "imu time %f", imu_end_time); 
            }
            if (imu_end_time > crop_pile_end_time){ // return if there is no lidar msg to work with or the imu msgs are ahead
                // RCLCPP_INFO(get_logger(), "Returned because late lidar msg.." );
                return;
            }
            double imu_start_time = toSec(imu_buffer[0]->header.stamp);
            // RCLCPP_INFO(get_logger(), "first point time %f", crop_point_stamps.front()); 

            // double imu_start_time = toSec(imu_buffer.front()->header.stamp); 
            // RCLCPP_INFO(get_logger(), "IMU buffer is flushed?"); 
            // RCLCPP_INFO(get_logger(), "IMU time interval %f start %f end %f", imu_end_time - imu_start_time, imu_start_time, imu_end_time); 
            // get the end time of the imu msg


            // find index of first and last point for the new subframe
            // naive attempt: assume first index is always 0
            size_t first_index = 0;
            size_t last_index = 0;
            
            // new_subframe->clear();
            pcl::PointCloud<PointType>::Ptr new_subframe = boost::make_shared<pcl::PointCloud<PointType>>();
            for  (int i = 0; crop_point_stamps[i] < imu_end_time; i++){
                // RCLCPP_INFO(get_logger(), "point time %f size %i index %i", crop_point_stamps[i], crop_point_stamps.size(), i); 
                // subframe->points.push_back(crop_pile->points[i]);
                last_index = i;
            }
            if (last_index > crop_pile->points.size()) // this should not happen but is a failsafe
            { 
                return;
            }

            double subframe_dt = crop_point_stamps[last_index] - crop_point_stamps[first_index];

            // put the points that are in the timeframe of the imu message inside the a new subframe
            new_subframe->points.insert(new_subframe->points.begin(), crop_pile->points.begin() + first_index, crop_pile->points.begin() + last_index +1);
            // RCLCPP_INFO(get_logger(), "Points are added to the subframe? first and last index %i, %i", first_index, last_index); 
            // subframe->points.resize(last_index);

            // create and add subframe relative point deltas
            for (size_t i=0; i < new_subframe->points.size(); i++){
                // subframe->points[i].intensity = floor(subframe->points[i].intensity);
                new_subframe->points[i].intensity = floor(new_subframe->points[i].intensity) + subframe_dt * i / (new_subframe->points.size()) + (crop_point_stamps[first_index] - imu_start_time);
            }


            synchronized_imu_subframes_.push_back(make_pair(imu_buffer.front(), new_subframe));
            sub_frame_count++;
            // RCLCPP_INFO(get_logger(), "Here?");

            // clean up crop_pile
            // RCLCPP_INFO(get_logger(), "Points in crop pile stamps %i", crop_point_stamps.size());
            // RCLCPP_INFO(get_logger(), "Points in crop pile %i", crop_pile->points.size());

            RCLCPP_INFO(get_logger(), "First and last index: %i, %i, points stamps: %f %f, subframe count added: %i", first_index, last_index, crop_point_stamps[first_index], crop_point_stamps[last_index], sub_frame_count);

            // RCLCPP_INFO(get_logger(), "Points in pile before erase: %i", crop_pile->points.size());
            crop_point_stamps.erase(crop_point_stamps.begin() + first_index, crop_point_stamps.begin() + last_index +1);
            crop_pile->points.erase(crop_pile->points.begin() + first_index, crop_pile->points.begin() + last_index +1);

            // RCLCPP_INFO(get_logger(), "Points in pile after erase: %i", crop_pile->points.size());
            // RCLCPP_INFO(get_logger(), "Pile first stamp: %f, pile last stamp %f", crop_point_stamps[0], crop_point_stamps[crop_point_stamps.size() - 1]);
            // *crop_pile = *new_crop_pile;

            // RCLCPP_INFO(get_logger(), "Points in subframe %i", last_index);
            // RCLCPP_INFO(get_logger(), "Points in crop pile %i", crop_pile->points.size());

            
            //  rotate to body frame incase imu and lidar are not in the same frame
            // derotateCloud(*cloud_in, *cloud_in); // if IMU and Lidar Axes are not aligned
        }


        void imuDataHandler(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
        {
            // put data into buffer back
            RCLCPP_INFO_ONCE(get_logger(),"First IMU message recieved..");
            // apply offset
            // double time = toSec(imu_msg->header.stamp);
            // time += sync_offset_ms_ *1e-3;
            // imu_msg->header.stamp = toStamp(time);

            double imu_msg_time = toSec(imu_msg->header.stamp);
            if (last_imu_time < imu_msg_time){
                // RCLCPP_INFO(get_logger(), "imu msg added to buffer timestamp: %f", imu_msg_time);
                ins.addImuMsgToBuffer(imu_msg);
                imu_buffer.push_back(imu_msg);
                last_imu_time = imu_msg_time;
            }


            // if (!initial_pose_recieved) {
            if (!first_lidar_received ) { // makes it keep update the pose until a lidar msg is recieved
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for first LiDAR msg..");
                initial_pose.orientation.w() = imu_msg->orientation.w;
                initial_pose.orientation.x() = imu_msg->orientation.x;
                initial_pose.orientation.y() = imu_msg->orientation.y;
                initial_pose.orientation.z() = imu_msg->orientation.z;
                initial_pose_recieved = true;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Initial attitude updated q: %f %f %f %f", initial_pose.orientation.w(), initial_pose.orientation.x(), initial_pose.orientation.y(), initial_pose.orientation.z());
                // INS note: send initial pose to external INS
                // ins.updateInitialPose(initial_pose);
            }

            addFrameToCropPile();
            syncCropLidar();

        }


        void publishCurrentCloud()
        {   
            sensor_msgs::msg::PointCloud2 msgs;
            pcl::PointCloud<PointType> transformed_cloud;
            // transformed_cloud = 
            // pcl::transformPointCloudWithNormals<PointType>(*cloud_in_ds, transformed_cloud, preint_residual.translation, preint_residual.rotation);
            // pcl::transformPointCloudWithNormals<PointType>(*cloud_in_ds, transformed_cloud, last_odometry_pose.position, last_odometry_pose.orientation);
            
            transformed_cloud = *cloud_in;
            // #ifndef __INTELLISENSE__ 
            // pcl::toROSMsg(transformed_cloud, msgs);
            // #endif
            toROSMsg(transformed_cloud, msgs);
            msgs.header.stamp = cloud_header.stamp;
            msgs.header.frame_id = frame_id;
            pointcloud_pub->publish(msgs);
            // RCLCPP_INFO(get_logger(), "Header: %s,  time of PCL: %f, num points: %i", cloud_header.frame_id, time_new_cloud.nanoseconds()/1e9, cloud_in->points.size());
        }

        void publishCurrentSubframe(double timestamp)
        {   
            sensor_msgs::msg::PointCloud2 msgs;
            // pcl::PointCloud<PointType> transformed_cloud;
            // pcl::transformPointCloudWithNormals<PointType>(*, transformed_cloud, last_odometry_pose.position, last_odometry_pose.orientation);

            toROSMsg(*current_subframe, msgs);
            // msgs.header.stamp = cloud_header.stamp;
            msgs.header.stamp = toHeaderStamp(timestamp);
            msgs.header.frame_id = frame_id;
            pointcloud_pub->publish(msgs);
            // RCLCPP_INFO(get_logger(), "Header: %s,  time of PCL: %f, num points: %i", cloud_header.frame_id, time_new_cloud.nanoseconds()/1e9, cloud_in->points.size());
        }

        void publishCurrentFullCloud()
        {   
            sensor_msgs::msg::PointCloud2 msgs;
            pcl::PointCloud<PointType> transformed_cloud;
   
            
            transformed_cloud = *cloud_in;
            // #ifndef __INTELLISENSE__ 
            // pcl::toROSMsg(transformed_cloud, msgs);
            // #endif
            toROSMsg(transformed_cloud, msgs);
            msgs.header.stamp = cloud_header.stamp;
            msgs.header.frame_id = frame_id;
            fullpointcloud_pub->publish(msgs);
            // RCLCPP_INFO(get_logger(), "Header: %s,  time of PCL: %f, num points: %i", cloud_header.frame_id, time_new_cloud.nanoseconds()/1e9, cloud_in->points.size());
        }

        void publishPriorCloud(Eigen::Matrix4f transformation)
        {   
            sensor_msgs::msg::PointCloud2 msgs;
            pcl::PointCloud<PointType> transformed_cloud;
            pcl::transformPointCloudWithNormals<PointType>(*cloud_in_ds, transformed_cloud, transformation);
            #ifndef __INTELLISENSE__ 
            pcl::toROSMsg(transformed_cloud, msgs);
            #endif
            msgs.header.stamp = cloud_header.stamp;
            msgs.header.frame_id = frame_id;

            pointcloudprior_pub->publish(msgs);
        }
        
        void publishKeyframeCloud()
        {   
            sensor_msgs::msg::PointCloud2 msgs;
            #ifndef __INTELLISENSE__ 
            pcl::toROSMsg(*cloud_keyframe, msgs);
            #endif
            msgs.header.stamp = cloud_header.stamp;
            // msgs.header.frame_id = frame_id;
            msgs.header.frame_id = frame_id;

            keyframe_cloud_pub->publish(msgs);
            // RCLCPP_INFO(get_logger(), "Header: %s,  time of PCL: %f, num points: %i", cloud_header.frame_id, time_new_cloud.nanoseconds()/1e9, cloud_in->points.size());
        }

        void publishLocalMap()
        {
            sensor_msgs::msg::PointCloud2 msgs;
            toROSMsg(*local_map, msgs);

            msgs.header.stamp = toHeaderStamp(current_subframe_time); 
            msgs.header.frame_id = "lidar_odom";
            localcloud_pub->publish(msgs);
            RCLCPP_INFO(get_logger(), "where?!");
            RCLCPP_INFO(get_logger(), "Local map published!");

        }


        void initializeParameters()
        {
            RCLCPP_INFO(get_logger(), "Initializing Parameters..");

            last_odometry_pose.orientation = Eigen::Quaterniond::Identity();
            observer_state.ori = Eigen::Quaterniond::Identity();
            preint_state.ori = Eigen::Quaterniond::Identity();

            last_odometry_pose.position = Eigen::Vector3d::Zero();
            last_odometry_pose.matrix = Eigen::Matrix4d::Identity();

            registration_transformation.matrix = Eigen::Matrix4d::Identity();
            registration_transformation.rotation = Eigen::Quaterniond::Identity();
            registration_transformation.translation = Eigen::Vector3d::Zero();

            last_odometry_transformation.rotation = Eigen::Quaterniond::Identity();
            last_odometry_transformation.translation = Eigen::Vector3d::Zero();

            keyframe_to_odometry_transformation.rotation = Eigen::Quaterniond::Identity();
            keyframe_to_odometry_transformation.translation = Eigen::Vector3d::Zero();
            
            preint_transformation_guess.rotation = Eigen::Quaterniond::Identity();
            preint_transformation_guess.translation = Eigen::Vector3d::Zero();
            preint_residual.rotation = Eigen::Quaterniond::Identity();
            preint_residual.translation = Eigen::Vector3d::Zero();

            ins_pose_new.orientation = Eigen::Quaterniond::Identity();
            ins_pose_new.position = Eigen::Vector3d::Zero();
            ins_pose.orientation = Eigen::Quaterniond::Identity();
            ins_pose.position = Eigen::Vector3d::Zero();
            ins_relative.rotation = Eigen::Quaterniond::Identity();
            ins_relative.translation = Eigen::Vector3d::Zero();

            // last_odometry_transformation = Eigen::Matrix4d::Identity();
            odometry_transformation_guess = Eigen::Matrix4d::Identity();
            init_guess = Eigen::Matrix4f::Identity();

            average_translation = Eigen::Vector3d::Zero();
            recent_translations.push_back(average_translation);

            // get_parameter("ds_voxel_size", ds_voxel_size_);

            
            odom.header.frame_id = "odom";
            odom.child_frame_id = frame_id;

            transformation_geommsg.header.frame_id = "odom";

            // // ds_voxel_size = 0.1f;
            // ds_voxel_size = 0.2f;
            // RCLCPP_INFO(get_logger(), "ds_voxel_size in function is: %f", ds_voxel_size_);
            down_size_filter.setLeafSize(ds_voxel_size_, ds_voxel_size_, ds_voxel_size_);
            down_size_filter_local_map.setLeafSize(ds_voxel_size_lc_, ds_voxel_size_lc_, ds_voxel_size_lc_);
            down_size_filter_keyframe_map.setLeafSize(ds_voxel_size_kf_, ds_voxel_size_kf_, ds_voxel_size_kf_);

            latest_frame_idx = 0;
            latest_keyframe_idx = 0;

            max_frames = local_map_init_frames_count_;
            if (local_map_width_ > 0){
                max_frames = local_map_width_;
            }

            system_initialized = false;
            system_start_time = run_clock.now().seconds();

        }


        void initializeSystem()
        {   
            RCLCPP_INFO(get_logger(), "Initializing System..");

            // make ESTIMATE INITIAL ORIENTATION function
            if (initial_pose_recieved){
                RCLCPP_INFO(get_logger(), "Using Initial Attitude quaternion from Madgwick:  q_init: %f, %f, %f, %f",initial_pose.orientation.w(), initial_pose.orientation.x(), initial_pose.orientation.y(), initial_pose.orientation.z() );
                last_odometry_pose.orientation =            Eigen::Quaterniond(initial_pose.orientation);
                last_odometry_pose.matrix =                 Eigen::Matrix4d::Identity();
                last_odometry_pose.matrix.block<3,3>(0,0) = Eigen::Matrix3d(last_odometry_pose.orientation);
                observer_state.ori  =                       Eigen::Quaterniond(initial_pose.orientation);
                preint_state.ori    =                       Eigen::Quaterniond(initial_pose.orientation);
                preint_anchor.ori   =                       Eigen::Quaterniond(initial_pose.orientation);
                preint_transformation_guess.rotation =      Eigen::Quaterniond(initial_pose.orientation);
                keyframe_pose.orientation =                 Eigen::Quaterniond(initial_pose.orientation);

                ins.initializeInitialPose(initial_pose.orientation);
            } else {
                RCLCPP_INFO(get_logger(), "No Initial attitude determined, assuming identity pose..");
                // last_odometry_pose.orientation = Eigen::Quaterniond::Identity();
                // observer_state.ori = Eigen::Quaterniond::Identity();
                // preint_state.ori = Eigen::Quaterniond::Identity();
                // preint_anchor.ori = Eigen::Quaterniond::Identity();
            }

            
            // RCLCPP_INFO(get_logger(), "Max frames in local map is %i", max_frames);

            last_odometry_pose.time = current_scan_time;

            // derotateCloud(*cloud_in, *cloud_in);

            // pcl::transformPointCloudWithNormals<PointType>(*cloud_in, *cloud_in, last_odometry_pose.matrix);

            // *global_cloud += *cloud_in;


            // publishCurrentCloud();

            // publishGlobalCloud();

            // saveRunningData();

            system_initialized = true;
            RCLCPP_INFO(get_logger(), "First frame initialized...");
        }

        void initializeScanmatcher()
        {
            RCLCPP_INFO(get_logger(), "Initializing Scanmatcher..");

            string method_name;
            // if (scan_matching_method_ == 3) {

                // pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr
                // ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
                // pclomp::NormalDistributionsTransform<PointType, PointType>::Ptr
                // ndt(new pclomp::NormalDistributionsTransform<PointType, PointType>());
                // ndt->setResolution(0.5);
                // ndt->setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
                // ndt->setStepSize(0.2);
                // // ndt_omp
                // // ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
                // ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
                // // if (ndt_num_threads > 0) {ndt->setNumThreads(ndt_num_threads);}
                // ndt->setNumThreads(8);
                // ndt->setTransformationEpsilon(1e-3);
                // ndt->setEuclideanFitnessEpsilon(1e-5);
                // ndt->setMaximumIterations(icp_max_iterations_);


                // typedef pcl::registration::TransformationEstimationPointToPlane<PointType, PointType> PointToPlane;
                // boost::shared_ptr<PointToPlane> p2pl(new PointToPlane);
                // ndt->setTransformationEstimation(p2pl);

                // registration_ = ndt;
                // method_name = "NDT OMP";

            // } else if (scan_matching_method_ == 1 || scan_matching_method_ == 2) {
            //     // typedef pcl::registration::TransformationEstimationPointToPlane<PointType, PointType> PointToPlane;

            //     boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>>
            //     gicp(new pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>());
            //     method_name = "GICP OMP";
            //     // if (scan_matching_method_ == 2) {1
            //         boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<PointType, PointType>>
            //         gicp(new pcl::GeneralizedIterativeClosestPoint<PointType, PointType>());
            //         method_name = "GICP";
            //     } 
            //     // boost::shared_ptr<PointToPlane> p2pl(new PointToPlane);
            //     // gicp->setTransformationEstimation(p2pl);

            //     gicp->setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
            //     gicp->setMaximumOptimizerIterations(500);
            //     gicp->setMaximumIterations(icp_max_iterations_);
            //     gicp->setCorrespondenceRandomness(25);
            //     gicp->setUseReciprocalCorrespondences(true);
            //     gicp->setTransformationEpsilon(1e-4);
            //     // gicp->setTransformationRotationEpsilon(cos(0.001));
            //     // max iterations... other settings?

            //     typedef pcl::registration::TransformationEstimationPointToPlane<PointType, PointType> PointToPlane;
            //     boost::shared_ptr<PointToPlane> p2pl(new PointToPlane);
            //     gicp->setTransformationEstimation(p2pl);

            //     // gicp->search

            //     registration_ = gicp;



            typedef pcl::registration::TransformationEstimationSVD<PointType, PointType, float> TransformEstimator;
            boost::shared_ptr<TransformEstimator> transform_estimator(new TransformEstimator);


            // } else
            if (scan_matching_method_ == 0 ){
                boost::shared_ptr<pcl::IterativeClosestPointWithNormals<PointType, PointType, float>> icp(new pcl::IterativeClosestPointWithNormals<PointType, PointType>());
                // typedef pcl::registration::TransformationEstimationPointToPlane<PointType, PointType> PointToPlane;
                // typedef pcl::registration::TransformationEstimationPointToPlaneLLS<PointType, PointType> PointToPlane;
                typedef pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<PointType, PointType, float> PointToPlane;
                boost::shared_ptr<PointToPlane> p2pl(new PointToPlane);
                icp->setTransformationEstimation(p2pl);

                icp->setMaxCorrespondenceDistance((float)icp_max_correspondence_distance_);
                // icp->setUseReciprocalCorrespondences(true);
                // icp->setUseSymmetricObjective(true);
                // icp->setEnforceSameDirectionNormals(true);
                icp->setMaximumIterations(icp_max_iterations_);
                icp->setTransformationEpsilon(1e-6);


                registration_ = icp;
                method_name = "ICP point-2-plane";

            } else {
                RCLCPP_INFO(get_logger(),"Defaulted to ICP p2p scan matching");
                // boost::shared_ptr<pcl::IterativeClosestPointNonLinear<PointType, PointType>> icp(new pcl::IterativeClosestPointNonLinear<PointType, PointType>());
                boost::shared_ptr<pcl::IterativeClosestPoint<PointType, PointType, float>> icp(new pcl::IterativeClosestPointNonLinear<PointType, PointType>());
                icp->setTransformationEstimation(transform_estimator);
                icp->setMaxCorrespondenceDistance((float)icp_max_correspondence_distance_);
                // icp->setUseReciprocalCorrespondences(true);
                // icp->setUseSymmetricObjective(true);
                // icp->setEnforceSameDirectionNormals(true);
                icp->setMaximumIterations(icp_max_iterations_);
                icp->setTransformationEpsilon(1e-4);
                

                registration_ = icp;
                method_name = "ICP point-2-point";

            }
            // rej->setInputCloud<PointType>(keypoints_src);
            // rej->setInputTarget<PointType>(keypoints_tgt);
            // rej.setInputCorrespondences (all_correspondences);

            pcl::registration::CorrespondenceRejectorDistance::Ptr rej(new pcl::registration::CorrespondenceRejectorDistance);
            rej->setMaximumDistance(icp_max_correspondence_distance_);
            registration_->addCorrespondenceRejector(rej);
            // registration_->corre

            RCLCPP_INFO(get_logger(), "Scan Match Registration method is: %s", method_name.c_str()); // put here the algorithm that is chosen, maybe in red color
        }

        


        void allocateMemory()
        {
            RCLCPP_INFO(get_logger(), "Allocating Point Cloud Memory..");

            cloud_in.reset(new pcl::PointCloud<PointType>());
            cloud_in_ds.reset(new pcl::PointCloud<PointType>());
            // cloud_prev.reset(new pcl::PointCloud<PointType>());
            // cloud_prev_ds.reset(new pcl::PointCloud<PointType>());

            keyframe_map.reset(new pcl::PointCloud<PointType>());
            keyframe_map_ds.reset(new pcl::PointCloud<PointType>());
            local_map.reset(new pcl::PointCloud<PointType>());
            local_map_ds.reset(new pcl::PointCloud<PointType>());
            // reduced_global_map.reset(new pcl::PointCloud<PointType>());
            // long_term_map.reset(new pcl::PointCloud<PointType>());
            target_map.reset(new pcl::PointCloud<PointType>());

            odometry_pose_info.reset(new pcl::PointCloud<PoseInfo>()); // positions of the odometry as clouds, enables fast knn search for nearby locations
            keyframe_pose_info.reset(new pcl::PointCloud<PoseInfo>());
            // odometry_pose_positions.reset(new pcl::PointCloud<pcl::PointXYZI>());
        }

    



        // Eigen::Matrix4d cloudRegisration(boost::shared_ptr<pcl::PointCloud<PointType>> source, const boost::shared_ptr<pcl::PointCloud<PointType>> target)
        Eigen::Matrix4d cloudRegisration(boost::shared_ptr<pcl::PointCloud<PointType>> source)
        {
            // rclcpp::Clock system_clock;
            

            registration_->setInputSource(source);
            // RCLCPP_INFO(get_logger(),"source size %i", source->points.size());
            // rclcpp::Time time_covcalc_start = system_clock.now();
            // registration_->setInputTarget(target);
            // rclcpp::Time time_covcalc_end = system_clock.now();
            // RCLCPP_INFO(get_logger(), "Covariance calculation time: %fms", time_covcalc_end.seconds()*1000.0 - time_covcalc_start.seconds()*1000.0);
            
            pcl::PointCloud<PointType>::Ptr aligned_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
            rclcpp::Time time_align_start = system_clock.now();
            // RCLCPP_INFO(get_logger(), "Problemin align?");
            registration_->align(*aligned_cloud, init_guess);
            // RCLCPP_INFO(get_logger(), "No..");
            rclcpp::Time time_align_end = system_clock.now();

            Eigen::Matrix4d registration_transform_double = registration_->getFinalTransformation().cast<double>();

            
            pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences);
            pcl::CorrespondencesPtr remaining_correspondences(new pcl::Correspondences);
            pcl::IndicesPtr rejected_indices(new pcl::Indices);

            pcl::registration::CorrespondenceEstimation<PointType, PointType> est;
            est.setInputSource(aligned_cloud);
            est.setInputTarget(registration_->getInputTarget());
            est.determineCorrespondences(*all_correspondences);

            // Apply the correspondence rejectors manually to count used correspondences in the matching
            std::vector<pcl::registration::CorrespondenceRejector::Ptr> rejectors = registration_->getCorrespondenceRejectors();
            int number_of_rejectors = rejectors.size();
            for (auto& rejector : rejectors)
            {
                rejector->setInputCorrespondences(all_correspondences);
                // pcl::CorrespondencesPtr remaining_correspondences(new pcl::Correspondences);
                rejector->getCorrespondences(*remaining_correspondences);
                rejector->getRejectedQueryIndices(*remaining_correspondences, *rejected_indices);
                all_correspondences = remaining_correspondences;
            }


            icp_fitness = registration_->getFitnessScore();
            float correspondence_part =  (float)remaining_correspondences->size() / (float)source->points.size();

            registration_process_time = time_align_end.seconds()*1000.0 - time_align_start.seconds()*1000.0;
            RCLCPP_INFO(get_logger(), "----  REGISTRATION ----");
            RCLCPP_INFO(get_logger(), "Registration converged: %i,  fitness: %f, time: %fms", registration_->hasConverged(), icp_fitness, registration_process_time);
            RCLCPP_INFO(get_logger(), "Size of source %i, size of target %i ", source->points.size(), registration_->getInputTarget()->points.size());
            RCLCPP_INFO(get_logger(), "Size of correspondences %i, part %f, number of rejectors %i", remaining_correspondences->size(), correspondence_part, number_of_rejectors);

            // add rejected points to the local map ie. expanding the map with the point that are not represented in the current map
            if (correspondence_part < 1.0){ // if there is more than 25% rejected then add those to the local map
                // Create the extraction object
                pcl::ExtractIndices<PointType> extract;
                extract.setInputCloud(aligned_cloud);
                extract.setIndices(rejected_indices);

                pcl::PointCloud<PointType>::Ptr extracted_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
                extract.filter(*extracted_cloud);
                *local_map += *extracted_cloud;
                setRegistrationTarget(local_map);
                publishLocalMap();
                RCLCPP_INFO(get_logger(), "added %i points to local map! count indices %i", extracted_cloud->points.size(), rejected_indices->size());
                
            }

            return registration_transform_double;
        }

        // void scanMatchRegistration(boost::shared_ptr<pcl::PointCloud<PointType>> src, const boost::shared_ptr<pcl::PointCloud<PointType>> tgt)
        void scanMatchRegistration(boost::shared_ptr<pcl::PointCloud<PointType>> src)
        {
            pcl::PointCloud<PointType>::Ptr source = boost::make_shared<pcl::PointCloud<PointType>>(*src);
            
            // Eigen::Matrix4d K;
            // Eigen::Matrix4d K_i;
            // K = keyframe_pose.matrix;
            // K_i = K.inverse();
            // pcl::transformPointCloudWithNormals<PointType>(*source, *source, K_i);

            RCLCPP_INFO(get_logger(), "Starting scan matching..");
            Eigen::Matrix4d registration_transform_matrix;
            registration_transform_matrix = cloudRegisration(source);
            // registration_transform_matrix = K * registration_transform_matrix * K_i;


            inertialConstrainTransformation(registration_transform_matrix);

            cout << registration_transform_matrix << "\n";

            // registration_transformation.rotation = Eigen::Quaterniond(registration_transform_matrix.block<3,3>(0,0)).normalized();
            registration_transformation.matrix = registration_transform_matrix;
            // registration_transformation.matrix.block<3,1>(0,3) += centroid3;
            registration_transformation.rotation = Eigen::Quaterniond(registration_transform_matrix.block<3,3>(0,0));
            registration_transformation.translation = Eigen::Vector3d(registration_transform_matrix.block<3,1>(0,3));
        }

        void inertialConstrainTransformation(Eigen::Matrix4d & transformation_in)
        {
            Eigen::Vector3d vel = transformation_in.block<3,1>(0,3) / scan_dt; // velocity in m/s
            double speed = vel.norm();
            if ( speed > 1.0){
                transformation_in.block<3,1>(0,3) = vel/speed * scan_dt;
            }

        }

        void setRegistrationTarget(boost::shared_ptr<pcl::PointCloud<PointType>> tgt)
        {
            pcl::PointCloud<PointType>::Ptr target = boost::make_shared<pcl::PointCloud<PointType>>(*tgt);
            // pcl::copyPointCloud(*tgt, *target);
            // Eigen::Matrix4d T;
            // T = keyframe_pose.matrix.inverse();
            // cout <<last_odometry_pose.matrix <<"\n" << T << "\n";
            // pcl::transformPointCloudWithNormals<PointType>(*target, *target, T);
            registration_->setInputTarget(target);
        }
    
        void simpleConstraningObserverUpdate(const Transformation preint_transformation_guess, INSstate &preint_anchor)
        {
            double gamma_1, gamma_2, gamma_3, gamma_4, gamma_5;
            gamma_1 = gamma_1_ * scan_dt;
            gamma_2 = gamma_2_ * scan_dt;
            gamma_3 = gamma_3_ * scan_dt;
            gamma_4 = gamma_4_ * scan_dt;
            gamma_5 = gamma_5_ * scan_dt;

            
            observer_state = preint_anchor; // get the anchor in the local frame
            // RCLCPP_INFO(get_logger(), "Anchor before: pos %f %f %f vel %f %f %f", observer_state.pos.x(), observer_state.pos.y(), observer_state.pos.z(), observer_state.vel.x(),observer_state.vel.y(), observer_state.vel.z());
            // RCLCPP_INFO(get_logger(), "Anchor before : ori %f %f %f %f", observer_state.ori.w(), observer_state.ori.x(), observer_state.ori.y(), observer_state.ori.z());

            // the scan match residual are in local frame
            // Eigen::Quaterniond q_error = preint_residual.rotation;
            // Eigen::Vector3d p_error = preint_residual.translation;

            // calculated in world frame - here last odometry is the current one just establsihed by ICP
            Eigen::Quaterniond q_error = preint_transformation_guess.rotation.inverse() * last_odometry_pose.orientation; 
            Eigen::Vector3d p_error = (last_odometry_pose.position - preint_transformation_guess.translation); 

            preint_residual.rotation = q_error;
            preint_residual.translation = last_odometry_pose.orientation.matrix() * p_error; // saved in the body frame

            RCLCPP_INFO(get_logger(), "Observer: Translation residual: %f %f %f norm: %f", p_error.x(), p_error.y(), p_error.z(), p_error.norm());
            RCLCPP_INFO(get_logger(), "Observer: Rotational residual: %f %f %f %f", q_error.w(), q_error.x(), q_error.y(), q_error.z());

            Eigen::Vector3d q_e_vec = q_error.vec(); // the imaginary part of the quaternion is made to a vector
            // RCLCPP_INFO(get_logger(), "q_error calc: %f %f %f %f", q_error_calc.w(), q_error_calc.x(), q_error_calc.y(), q_error_calc.z());
            // Eigen::Quaterniond q_error = observer_state.ori.inverse() * last_odometry_pose.orientation;
            // Eigen::Vector3d p_error = last_odometry_pose.position - observer_state.pos;

            double qw = q_error.w();
            double abs_qw = abs(qw);
            int sgn_qw = 1;
            if (abs_qw != 0.0) // to avoid division with zero, if abs_qw = 0.0 sign is set positive 
                sgn_qw = qw/ abs_qw;

            Eigen::Quaterniond q_e(1.0 - abs_qw, sgn_qw * q_error.x(), sgn_qw * q_error.y(), sgn_qw * q_error.z());
            // RCLCPP_INFO(get_logger(), "q_e: %f %f %f %f", q_e.w(), q_e.x(), q_e.y(), q_e.z());

            // Eigen::Vector3d dynamic_model_deweight(1.0, 0.3, 0.05); // a ground based vehicle is not expected to move sideways or laterally
            Eigen::Vector3d dynamic_model_deweight(1.0, 1.0, 1.0); 
            Eigen::Vector3d z_deweight(1.0, 1.0, 0.1);
            // Eigen::Vector3d eps(0.0001,0.0001,0.0001);

            observer_state.ori           = addQuaternions(observer_state.ori,  multQuatByScalar(observer_state.ori * q_e, gamma_1)).normalized();
            observer_state.bias.ang     -=  gamma_2 * qw * q_e_vec;  
            // Eigen::Vector3d velocity_deweight(observer_state.vel);
            // velocity_deweight = (velocity_deweight.cwiseAbs()) / (velocity_deweight.cwiseAbs().maxCoeff() + 1e-5);
            // observer_state.pos          +=  gamma_3 * (p_error).cwiseProduct(observer_state.ori.matrix() * dynamic_model_deweight);
            observer_state.pos          +=  gamma_3 * p_error;
            observer_state.vel          +=  gamma_4 * (p_error).cwiseProduct(dynamic_model_deweight); // here it is corrected in world frame
            // observer_state.vel          +=  gamma_4 * (observer_state.ori.matrix().inverse() * p_error).cwiseProduct(dynamic_model_deweight); // here it is corrected in the local frame
            observer_state.bias.acc     -=  gamma_5 * observer_state.ori.matrix().inverse() * p_error;  // the error is rotated into local frame to update bias
            // observer_state.bias.acc     -=  gamma_5 *  p_error;  

            if (abs(observer_state.bias.ang[1])  > 1.0) { // attempt at debugging, this should not happen
                RCLCPP_INFO(get_logger(), "What is wrong?");
                RCLCPP_INFO(get_logger(), "q_res: %f %f %f %f", q_error.w(), q_error.x(), q_error.y(), q_error.z());
                RCLCPP_INFO(get_logger(), "q_e: %f %f %f %f", q_e.w(), q_e.x(), q_e.y(), q_e.z());
                observer_state.bias.ang[1] = 0.0;
            }   
            // RCLCPP_INFO(get_logger(), "Anchor after: pos %f %f %f vel %f %f %f", observer_state.pos.x(), observer_state.pos.y(), observer_state.pos.z(), observer_state.vel.x(),observer_state.vel.y(), observer_state.vel.z());
            // RCLCPP_INFO(get_logger(), "Anchor after : ori %f %f %f %f", observer_state.ori.w(), observer_state.ori.x(), observer_state.ori.y(), observer_state.ori.z());

            preint_anchor = observer_state;
            Eigen::Vector3d local_velocity(observer_state.ori.matrix().inverse() * observer_state.vel);
            RCLCPP_INFO(get_logger(), "Observer: local frame vel: %f %f %f", local_velocity.x(), local_velocity.y(), local_velocity.z());
            RCLCPP_INFO(get_logger(), "Observer: bias acc: %f %f %f", observer_state.bias.acc.x(), observer_state.bias.acc.y(), observer_state.bias.acc.z());
            RCLCPP_INFO(get_logger(), "Observer: bias ang: %f %f %f", observer_state.bias.ang.x(), observer_state.bias.ang.y(), observer_state.bias.ang.z());

        }

        bool getNextInBuffer()
        {
            // get next frame in buffer..
            if (cloud_queue.size() <= 0) { 
                return false; 
            } else {
                current_cloud_msg = cloud_queue.front(); // puts the next cloud to variable current
                cloud_queue.pop_front(); // removes element from the queue

                // set header id and timestamp
                cloud_header = current_cloud_msg.header; 
                cloud_header.frame_id = frame_id;
    
                time_new_cloud = current_cloud_msg.header.stamp;
                current_scan_time = toSec(time_new_cloud) + sync_offset_ms_ *1e-3;
                // next_scan_time = toSec(cloud_queue.front().header.stamp) + sync_offset_ms_ *1e-3; // obs this is not necassarily just current time + scan_dt due to holes in recordings

                if ( start_delay_s_ > (current_scan_time - first_lidar_msg_time)){
                    RCLCPP_INFO(get_logger(), "Buffer time: %f", current_scan_time - first_lidar_msg_time);
                    return false;
                }

                if (cloud_queue.size() > 5) {
                    RCLCPP_WARN(get_logger(), "Buffer size is: %i", cloud_queue.size());
                }
            } 

            // cloud_header = pcl_msg->header;
            fromROSMsg(current_cloud_msg, *cloud_in);

            
            // this scan dt is validated in the preprocesser why it is pulled from the last points intensity value
            scan_dt = cloud_in->points[cloud_in->points.size() -1].intensity - (int)cloud_in->points[cloud_in->points.size() -1].intensity;
            
            RCLCPP_INFO(get_logger(), "Processing frame index: %i, scan timestamp: %f dt: %f", latest_frame_idx, current_scan_time, scan_dt);
            
            derotateCloud(*cloud_in, *cloud_in); // if IMU and Lidar Axis are not aligned
            // rotate to body frame
            // pcl::transformPointCloudWithNormals<PointType>(*cloud_in, *cloud_in, last_odometry_pose.position, last_odometry_pose.orientation);

            return true;
        }

        bool pop_synchronized_data(std::pair<sensor_msgs::msg::Imu::SharedPtr, pcl::PointCloud<PointType>::Ptr>& data) 
        {
            if (!synchronized_imu_subframes_.empty()) {

                data = synchronized_imu_subframes_.front();
                RCLCPP_INFO(get_logger(), "Length of synced data buffer %i", synchronized_imu_subframes_.size() );
                // synchronized_imu_subframes_.erase(synchronized_imu_subframes_.begin());
                synchronized_imu_subframes_.pop_front();
                RCLCPP_INFO(get_logger(), "Data Popped succesfully..");
                return true;
            }
            // RCLCPP_INFO(get_logger(), "No Data Pop..");
            return false;  // Indicate that there was no data to pop
        }   

        bool getNextSubframe()
        {
            // get next frame in buffer..
            
            std::pair<sensor_msgs::msg::Imu::SharedPtr, pcl::PointCloud<PointType>::Ptr> synced_frame;
            if (pop_synchronized_data(synced_frame)){
                current_imu_msg = synced_frame.first;
                current_subframe = synced_frame.second;
                current_subframe_time = toSec(current_imu_msg->header.stamp);
                latest_frame_idx++;
                scan_dt = current_subframe->points[current_subframe->points.size() -1].intensity - (int)current_subframe->points[current_subframe->points.size() -1].intensity;
                RCLCPP_INFO(get_logger(), "Processing subframe index: %i, subframe(imu) timestamp: %f subframe dt: %f length %i", latest_frame_idx, current_subframe_time, scan_dt, synced_frame.second->points.size());
                return true;
            }
            return false;

        }

        void  buildInitialLocalMap()
        {
            // loops until a set time is passed from the first lidar scan, for instance the first 2 sec are stationary and therefore used to build an initial map.
            // This time interval should also be used to saturate bias estimates of the IMU.
            while (current_subframe_time - first_lidar_msg_time < 0.1 )
            {

                *local_map += *current_subframe; // transform to initial pose??
                RCLCPP_INFO(get_logger(), "building map..");
                while (!getNextSubframe()){ // loop catch  to wait for the next subframe 
                    // RCLCPP_INFO(get_logger(), ".. but caught in a trap");
                }
            }

            setRegistrationTarget(local_map);
            publishLocalMap();
        }


        void updateOdometryPose(Transformation update_transformation)
        {   
            Eigen::Matrix4d update_T, Last_T, Last_P, Key_P, next_P, DeltaT;
            update_T = Eigen::Matrix4d::Identity();
            Last_T = Eigen::Matrix4d::Identity();
            Last_P = Eigen::Matrix4d::Identity();
            Key_P = Eigen::Matrix4d::Identity();
            next_P = Eigen::Matrix4d::Identity();
            DeltaT = Eigen::Matrix4d::Identity();
            Last_P.block<3,1>(0,3) = last_odometry_pose.position;
            Last_P.block<3,3>(0,0) = Eigen::Matrix3d(last_odometry_pose.orientation);
            // Key_P.block<3,1>(0,3) = keyframe_poses.back().position;
            // Key_P.block<3,3>(0,0) = Eigen::Matrix3d(keyframe_poses.back().orientation);

            // update_T.block<3,1>(0,3) = update_transformation.translation;
            // update_T.block<3,3>(0,0) = Eigen::Matrix3d(update_transformation.rotation);
            update_T = update_transformation.matrix;
            DeltaT = update_transformation.matrix;
            // RCLCPP_INFO(get_logger(), "prior pose: pos %f %f %f ", preint_transformation_guess.translation.x(), preint_transformation_guess.translation.y(), preint_transformation_guess.translation.z());
            // if (use_preint_undistortion_){

            //     RCLCPP_INFO(get_logger(), "prior INS pose: pos %f %f %f ", preint_transformation_guess.translation.x(), preint_transformation_guess.translation.y(), preint_transformation_guess.translation.z());
            //     RCLCPP_INFO(get_logger(), "prior INS pose: ori %f %f %f %f", preint_transformation_guess.rotation.w(), preint_transformation_guess.rotation.x(), preint_transformation_guess.rotation.y(), preint_transformation_guess.rotation.z());
                
            //     Eigen::Matrix4d T_M;
            //     T_M = Eigen::Matrix4d::Identity();
            //     T_M.block<3,3>(0,0) = Eigen::Matrix3d(preint_transformation_guess.rotation);
            //     T_M.block<3,1>(0,3) = preint_transformation_guess.translation;
            //     // cloud_zeroing_transform = Eigen::Matrix4d::Identity();
                
            //     // this value is only used to save to file

            //     update_T = DeltaT * T_M; // DeltaT is the refining ICP transformation, T_M is the preintegrated guess
            //     // update_T = T_M * DeltaT ; // DeltaT is the refining ICP transformation, T_M is the preintegrated guess
            //     update_transformation.translation = update_T.block<3,1>(0,3);
            //     update_transformation.rotation = Eigen::Quaterniond(update_T.block<3,3>(0,0));

            //     next_P = update_T;
            // } else {
                // next_P = Key_P * update_T;
                // next_P = keyframe_pose.matrix * update_T ;
                // next_P = update_T * keyframe_pose.matrix;
                // update_T =  DeltaT * Key_P;
                // update_T =  Key_P * DeltaT;
                next_P = DeltaT * Last_P;


            // }

            // correct the cloud by the scan match delta
            // pcl::transformPointCloudWithNormals<PointType>(*cloud_in, *cloud_in, DeltaT);
            // pcl::transformPointCloudWithNormals<PointType>(*cloud_in_ds, *cloud_in_ds, DeltaT);
            
            // update_T.block<3,1>(0,3) = update_transformation.translation;
            // update_T.block<3,3>(0,0) = Eigen::Matrix3d(update_transformation.rotation);

            // RCLCPP_INFO(get_logger(), "key %f %f %f", translation_std_x, translation_std_y, translation_std_z);
            
            // Eigen::Vector3d rotated_translation = keyframe_poses.back().orientation * update_transformation.translation;
        
            Pose next_odometry_pose;
            // next_odometry_pose.position =   keyframe_poses.back().position  + rotated_translation;
            // next_odometry_pose.orientation = (update_transformation.rotation * keyframe_poses.back().orientation).normalized();
            next_odometry_pose.position =    next_P.block<3,1>(0,3);
            next_odometry_pose.orientation = Eigen::Quaterniond(next_P.block<3,3>(0,0));
            
            Last_T = Last_P.inverse() * next_P;
            last_odometry_transformation.translation = Last_T.block<3,1>(0,3);  // transformation between 2 consecutive odometry poses
            last_odometry_transformation.rotation = Eigen::Quaterniond(Last_T.block<3,3>(0,0)); // transformation between 2 consecutive odometry poses
            last_odometry_transformation.matrix = Last_T;
   

            last_odometry_pose.position = Eigen::Vector3d(next_odometry_pose.position);
            last_odometry_pose.orientation = Eigen::Quaterniond(next_odometry_pose.orientation.normalized());
            last_odometry_pose.velocity = last_odometry_transformation.translation / scan_dt; // translation is on local frame therefore this speed is in the local frame
            last_odometry_pose.time = current_subframe_time;
            last_odometry_pose.matrix = next_P;
            

            

            RCLCPP_INFO(get_logger(), "Posterior LO pose: pos %f %f %f ", last_odometry_pose.position.x(), last_odometry_pose.position.y(), last_odometry_pose.position.z());
            RCLCPP_INFO(get_logger(), "Posterior LO pose: vel %f %f %f ", last_odometry_pose.velocity.x(), last_odometry_pose.velocity.y(), last_odometry_pose.velocity.z());
            RCLCPP_INFO(get_logger(), "Posterior LO pose: ori %f %f %f %f", last_odometry_pose.orientation.w(), last_odometry_pose.orientation.x(), last_odometry_pose.orientation.y(), last_odometry_pose.orientation.z());
            
            Eigen::Vector3d ypr = quaternionToEulerAngles(last_odometry_pose.orientation) * 180.0 / M_PI;
            RCLCPP_INFO(get_logger(), "Posterior LO pose: ypr %f %f %f degrees", ypr.x(), ypr.y(), ypr.z());


            // keyframe_to_odometry_transformation.rotation = keyframe_pose.orientation.inverse() * last_odometry_pose.orientation;
            // keyframe_to_odometry_transformation.translation = keyframe_pose.orientation.inverse() * (last_odometry_pose.position - keyframe_pose.position);
        }

        void savePose()
        {
            // function to push new_pose to odometries, saved as quaternion and position vector

            // Eigen::Quaterniond reg_quarternion = last_odometry_pose.orientation; 
            // reg_quarternion.normalize();
            // Eigen::Vector3d reg_translation = last_odometry_pose.position;

            PoseInfo new_pose;
            new_pose.qw = last_odometry_pose.orientation.w();
            new_pose.qx = last_odometry_pose.orientation.x();
            new_pose.qy = last_odometry_pose.orientation.y();
            new_pose.qz = last_odometry_pose.orientation.z();
            new_pose.x = last_odometry_pose.position.x();
            new_pose.y = last_odometry_pose.position.y();
            new_pose.z = last_odometry_pose.position.z();
            new_pose.idx = odometry_pose_info->points.size(); // the size will be the index of the next pose
            new_pose.time = last_odometry_pose.time;

            odometry_pose_info->push_back(new_pose);

        }

        void publishOdometry()
        {
            // PoseInfo latestPoseInfo = odometry_pose_info->points[latest_keyframe_idx - 1];
            PoseInfo latestPoseInfo = odometry_pose_info->points[odometry_pose_info->points.size() -1 ];
            // odom.header.stamp = cloud_header.stamp;
            odom.header.stamp = toHeaderStamp(latestPoseInfo.time);
            odom.pose.pose.orientation.w = latestPoseInfo.qw;
            odom.pose.pose.orientation.x = latestPoseInfo.qx;
            odom.pose.pose.orientation.y = latestPoseInfo.qy;
            odom.pose.pose.orientation.z = latestPoseInfo.qz;
            odom.pose.pose.position.x = latestPoseInfo.x;
            odom.pose.pose.position.y = latestPoseInfo.y;
            odom.pose.pose.position.z = latestPoseInfo.z;

            odom.twist.twist.linear.x = last_odometry_pose.velocity.x();
            odom.twist.twist.linear.y = last_odometry_pose.velocity.y();
            odom.twist.twist.linear.z = last_odometry_pose.velocity.z();

            // vector<double> cov_diag{translation_std_x*translation_std_x, translation_std_y* translation_std_y, translation_std_z * translation_std_z, 0.001, 0.001, 0.001};
            // Eigen::MatrixXd cov_mat = createCovarianceEigen(cov_diag);
            // // Eigen::Matrix3d rot_mat = last_odometry_pose.block<3,3>(0,0);
            // Eigen::Matrix3d rot_mat(Eigen::Quaterniond(latestPoseInfo.qw, latestPoseInfo.qx, latestPoseInfo.qy, latestPoseInfo.qz ));
            // rotateCovMatrix(cov_mat, rot_mat);
            // setCovariance(odom.pose.covariance, cov_mat);
            // // set scan dt in the top right position (idx 5) of 6x6 covariance matrix
            // odom.pose.covariance[5] = scan_dt;

            // odom.twist.twist.linear.x // add the velocities in twist
            odometry_pub->publish(odom);


            // odom -> base_link transform
            geometry_msgs::msg::TransformStamped t_;
            // t_.header.stamp = this->get_clock()->now();
            t_.header.stamp = cloud_header.stamp;
            t_.header.frame_id = "odom";//"lidar_odom";
            t_.child_frame_id = "base_link"; // "livox_frame"

            t_.transform.rotation.w = odom.pose.pose.orientation.w;
            t_.transform.rotation.x = odom.pose.pose.orientation.x;
            t_.transform.rotation.y = odom.pose.pose.orientation.y;
            t_.transform.rotation.z = odom.pose.pose.orientation.z;
            t_.transform.translation.x = odom.pose.pose.position.x;            
            t_.transform.translation.y = odom.pose.pose.position.y;  
            t_.transform.translation.z = odom.pose.pose.position.z;
            tf_broadcaster_->sendTransform(t_);


            geometry_msgs::msg::PoseStamped poseStamped;
            poseStamped.header = odom.header;
            poseStamped.pose = odom.pose.pose;
            poseStamped.header.stamp = odom.header.stamp;
            path.header.stamp = odom.header.stamp;
            path.poses.push_back(poseStamped);
            // path.header.frame_id = frame_id;
            path.header.frame_id = "odom"; // "livox_frame"
            path_pub->publish(path);
        }

// This is the node "main" script

        void run()
        {

            // addFrameToCropPile();
            // syncCropLidar();



            if (!getNextSubframe()){
                return;
            }
            if (current_subframe->points.size() <= 0)
            {
                RCLCPP_INFO(get_logger(), "SKIPPED");
                return; // skip for now but use the INS to predict next position when implemented
            }
            
            // RCLCPP_INFO(get_logger(), "Is the synced pair retrieved?");
            


            // rclcpp::Clock frame_clock;
            // rclcpp::Clock missing_time_clock;
            // rclcpp::Time time_mystery_end;
            // rclcpp::Time time_frame_start = frame_clock.now();


            // double undistort_time{};

            // latest_frame_idx++;
            // RCLCPP_INFO(get_logger(), "Frame idx: %i", latest_frame_idx);

            // if (latest_frame_idx < (size_t)start_idx_ || (latest_frame_idx > (size_t)end_idx_ && end_idx_ > 0) ){
            //     RCLCPP_INFO(get_logger(), "Skipping frames not in interval %i to %i", start_idx_, end_idx_);
            //     return;
            // }

            if (!system_initialized){
                // save first pose and point cloud and initialize the system
                initializeSystem();
                buildInitialLocalMap();
                RCLCPP_INFO(get_logger(), "ISS LOAM system is initialized.");
                return;
            }


            // RCLCPP_INFO(get_logger(), "Subframe size %i", current_subframe->points.size());
            // RCLCPP_INFO(get_logger(), "local Map size %i", registration_->getInputTarget()->points.size());

            pcl::transformPointCloudWithNormals<PointType>(*current_subframe, *current_subframe, last_odometry_pose.matrix);
            scanMatchRegistration(current_subframe); // TODO: make a check for degenerate estimated transformation 
            RCLCPP_INFO(get_logger(), "Registration done");
            updateOdometryPose(registration_transformation);
            pcl::transformPointCloudWithNormals<PointType>(*current_subframe, *current_subframe, registration_transformation.matrix);
            RCLCPP_INFO(get_logger(), "Pose updated");
            savePose();

            RCLCPP_INFO(get_logger(), "pose is saved");

            publishOdometry(); 
            // rclcpp::Time time_mystery_start = frame_clock.now();
            RCLCPP_INFO(get_logger(), "pose is published");



            publishCurrentSubframe(current_subframe_time);

            
            RCLCPP_INFO(get_logger(), "count of used indices in registration (not sure if this is correct) %i ", registration_->getIndices()->size());



            
            
            // } else { // keyframe rejection ei. 
            // }
            
            // saveRunningData();
            

            // rclcpp::Time time_frame_end = frame_clock.now();
            // double frame_process_time = time_frame_end.seconds()*1000.0 - time_frame_start.seconds()*1000.0; 
            // double mystery_time = time_mystery_end.seconds()*1000.0 - time_mystery_start.seconds()*1000.0; 

            // double total_run_time = run_clock.now().seconds() - system_start_time;

            // RCLCPP_INFO(get_logger(), " -- Frame process time: This: %f ms Average: %f ms ---  Reg. difference: %f ms", frame_process_time, total_run_time/(double)latest_frame_idx *1000.0 ,  frame_process_time - registration_process_time );
            // // RCLCPP_INFO(get_logger(), " -- Frame process time ---  %f ms --- difference --- %f ms", frame_process_time, frame_process_time - registration_process_time );
            // RCLCPP_INFO(get_logger(), " -- mystery time:  %f ms --- Undistortion time: %f ms" , mystery_time, undistort_time);

            // double data_elapsed_time = current_scan_time - first_lidar_msg_time;
            // double real_time_factor = data_elapsed_time / total_run_time;

            // RCLCPP_INFO(get_logger(), "Real-Time Factor %f -- Processed time: %fs -- Total run Time: %fs" , real_time_factor, data_elapsed_time, total_run_time);


        }



};


int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto lidar_odometry_node = std::make_shared<LidarOdometry>();
    executor.add_node(lidar_odometry_node);


    // rclcpp::spin(std::make_shared<LidarOdometry>());
    executor.spin();
    rclcpp::shutdown();
    return 0;


}