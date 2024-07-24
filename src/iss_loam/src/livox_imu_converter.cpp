
#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/msg/imu.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <pcl/common/common.h>
// #include <pcl/common/transforms.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/conversions.h>
// #include <pcl/filters/filter.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl_conversions/pcl_conversions.h>




#include <memory>
#include <cstdio>
#include <cmath>
#include <queue>
#include <vector>
// #include <eigen>

using namespace std;
// using namespace Eigen;

using std::placeholders::_1;

// struct PoseInfo
// {
//     double x;
//     double y;
//     double z;
//     double qw;
//     double qx;
//     double qy;
//     double qz;
//     int idx;
//     double time;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT (PoseInfo,
//                                    (double, x, x) (double, y, y) (double, z, z)
//                                    (double, qw, qw) (double, qx, qx) (double, qy, qy) (double, qz, qz)
//                                    (int, idx, idx) (double, time, time)
//                                    )



// put the following in a genereal header..
// typedef pcl::PointXYZINormal PointType; // need to calculate and assign normals
// typedef pcl::PointXYZINormal PointType; // need to calculate and assign normals

class IMUConverter : public rclcpp::Node
{
    private:
        // rclcpp::CallbackGroup::SharedPtr sub1_cb_group_;
        // rclcpp::CallbackGroup::SharedPtr sub2_cb_group_;

        // rclcpp::CallbackGroup::SharedPtr run_cb_group_;
        // rclcpp::TimerBase::SharedPtr run_timer;

        


        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr keyframe_odom_sub;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
        // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
        // rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

        // rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub;
     
      
        // rclcpp::Time time_latest_cloud;


        // NOTE: make filter or mechanization do the gravity estimate !!!


        // double imu_dt_; 
        double g_scale_; // gravity scalar value


        int step{};



        Eigen::Vector3d acceleration;
        Eigen::Vector3d acceleration_measured;
        Eigen::Vector3d angular_velocity;
        Eigen::Vector3d angular_velocity_measured;

        Eigen::Vector3d jerk;
        Eigen::Vector3d angular_acceleration;

        // double alpha_;
        // double beta_;


        std::string imu_topic_in_;
        std::string imu_topic_out_;


        // deque<sensor_msgs::msg::Imu::SharedPtr> imu_delay_buffer;

        int imu_step_delay_;


        // std_msgs::msg::Header imu_data_header;
        sensor_msgs::msg::Imu::SharedPtr filtered_imu_data;

    
    
    public:
        IMUConverter() // constructer
        : Node("livox_imu_converter")
        {   
            // declare_parameter("imu_dt", 0.01);
            // get_parameter("imu_dt", imu_dt_);

            declare_parameter("g_scale", 9.815);
            get_parameter("g_scale", g_scale_);
            
            declare_parameter("imu_topic_in", "/livox/imu");
            get_parameter("imu_topic_in", imu_topic_in_);

            declare_parameter("imu_topic_out", "/imu/data_raw_livox");
            get_parameter("imu_topic_out", imu_topic_out_);

            RCLCPP_INFO(get_logger(),"Livox IMU re-scaler - listning on topic: %s, republishing on: %s", imu_topic_in_.c_str(), imu_topic_out_.c_str());
            RCLCPP_INFO(get_logger(),"Scaling by g-value: %f", g_scale_);


            // sub1_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            // rclcpp::SubscriptionOptions options1;
            // options1.callback_group = sub1_cb_group_;


            imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_in_, 100, std::bind(&IMUConverter::imuDataHandler, this, _1));

            imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_out_, 100);
        
            

            initializeArrays();

        }
        ~IMUConverter(){}

        void initializeArrays()
        {   
            step = 0;
   
            acceleration << 0.0,0.0,0.0;
            jerk << 0.0,0.0,0.0;
            angular_velocity << 0.0,0.0,0.0;

        }

        void imuDataHandler(const sensor_msgs::msg::Imu::SharedPtr imu_data)
        {
            RCLCPP_INFO_ONCE(get_logger(), "First imu msg recieved..");
            step++;
            // // put data into buffer back
            // imu_delay_buffer.push_back(imu_data);

            // if (step < imu_step_delay_){ // if step is less than imu_step_delay return
            //     return;
            // }

            // // get data from buffer front
            // sensor_msgs::msg::Imu::SharedPtr delayed_imu_data = imu_delay_buffer.back();
            // imu_delay_buffer.pop_back();

            // imu_data_header = delayed_imu_data->header;
            // filtered_imu_data = *delayed_imu_data;


            filtered_imu_data = imu_data;

            acceleration_measured << filtered_imu_data->linear_acceleration.x,
                                     filtered_imu_data->linear_acceleration.y,
                                     filtered_imu_data->linear_acceleration.z;

            angular_velocity_measured << filtered_imu_data->angular_velocity.x,
                                         filtered_imu_data->angular_velocity.y,
                                         filtered_imu_data->angular_velocity.z;


            updateAcceleration();
            updateAngularVelocity();


            publish();

        }   

        void updateAcceleration()
        {   
            
            acceleration = acceleration_measured * g_scale_;

        }

        void updateAngularVelocity()
        {   

            angular_velocity = angular_velocity_measured;
        }
        
        void publish()
        {
            // data
            filtered_imu_data->linear_acceleration.x = acceleration[0];
            filtered_imu_data->linear_acceleration.y = acceleration[1];
            filtered_imu_data->linear_acceleration.z = acceleration[2];

            filtered_imu_data->angular_velocity.x = angular_velocity[0];
            filtered_imu_data->angular_velocity.y = angular_velocity[1];
            filtered_imu_data->angular_velocity.z = angular_velocity[2];

            imu_pub->publish(*filtered_imu_data);
        }


};

int main(int argc, char * argv[])
{
    // rclcpp::init(argc, argv);
    // auto convert_node = std::make_shared<IMUConverter>();
    // rclcpp::executors::MultiThreadedExecutor executor;
    // executor.add_node(convert_node);
    
    // executor.spin();
    // rclcpp::shutdown();
    // return 0;


    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUConverter>());
    rclcpp::shutdown();
    return 0;
}