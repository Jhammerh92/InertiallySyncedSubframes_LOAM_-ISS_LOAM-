#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"
#include "iss_loam/utils/common.hpp"
// #include "common.h"

#include <sensor_msgs/msg/point_cloud2.hpp>
// #include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/msg/imu.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

// #include <cmath>
// #include <ctime>
// #include <array>
// #include <string>
#include <vector>
// #include <algorithm>
// #include <iostream>
// #include <fstream>
// #include <thread>
// #include <mutex>
#include <queue>
// #include <assert.h>

// typedef pcl::PointXYZI PointTypeNoNormals; // need to calculate and assign normals and the change to PointXYZINormals
// typedef pcl::PointXYZINormal PointType;
// // typedef pcl::PointXYZR PointTypewTime;

// using namespace std;
// using std::placeholders::_1;



class Preprocessing : public rclcpp::Node
{
private:
    rclcpp::CallbackGroup::SharedPtr subscriber_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_Lidar_cloud;
    rclcpp::TimerBase::SharedPtr process_timer;

    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;


    pcl::VoxelGrid<PointType> down_size_filter;
    // ros::Subscriber sub_imu;

    // ros::Publisher pub_surf;
    // ros::Publisher pub_edge;
    // ros::Publisher pub_cutted_cloud;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_surf;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_edge;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cutted_cloud;

    // int pre_num = 0;

    pcl::PointCloud<PointType>::Ptr lidar_cloud_in = boost::make_shared<pcl::PointCloud<PointType>>();
    // pcl::PointCloud<PointTypeWTime>::Ptr lidar_cloud_in = boost::make_shared<pcl::PointCloud<PointTypeWTime>>();
    pcl::PointCloud<PointTypeNoNormals>::Ptr lidar_cloud_in_no_normals = boost::make_shared<pcl::PointCloud<PointTypeNoNormals>>();
    // pcl::PointCloud<PointTypeWTime>::Ptr lidar_cloud_in_with_time = boost::make_shared<pcl::PointCloud<PointTypeWTime>>();
    pcl::PointCloud<PointType>::Ptr lidar_cloud_cutted = boost::make_shared<pcl::PointCloud<PointType>>();
    
    // pcl::StatisticalOutlierRemoval<PointT> sor;
    
    std_msgs::msg::Header cloud_header;

    vector<sensor_msgs::msg::Imu::SharedPtr> imu_buffer;
    size_t imu_buffer_max_size = 600;
    int idx_imu = 0;
    int latest_frame_idx = 0;
    int start_idx_ = 0;
    int end_idx_ = 0;
    double current_time_imu = -1;
    double lidar_zero_time = -1;

    deque<double> scan_dts;

    Eigen::Vector3d gyr_0;
    Eigen::Quaterniond q_iMU = Eigen::Quaterniond::Identity();
    bool first_imu = true;
    bool first_lidar = true;

    std::deque<sensor_msgs::msg::PointCloud2> cloud_queue;
    sensor_msgs::msg::PointCloud2 current_cloud_msg;

    double time_scan; // convert to rclcpp::Time?
    double time_scan_next; // convert to rclcpp::Time?
    // rclcpp::Time time_scan_next;

    // int N_SCANS = 6;
    // int H_SCANS = 4000;

    string frame_id = "lidar_odom";
    string lidar_source_topic_ = "";
    double runtime = 0;

    double frame_procces_time;
    // ---- ROSPARAMETERS -----

    double edge_threshold_;
    double surfaces_threshold_;


    // double normal_search_radius = 0.2; // small scale / indoors
    double pc_normal_search_radius_; // large scale / outdoors
    int pc_normal_knn_points_;

    double filter_close_points_distance_m_;
    double filter_max_intensity_;
    bool use_gyroscopic_undistortion_{};
    double pre_downsample_leaf_size_;

    int remove_statistical_outliers_knn_points_;
    double remove_statistical_outliers_std_mul_;
    double remove_transition_outliers_cos_angle_threshold_;
    double normalize_intensities_reference_range_;
    int calculate_point_curvature_kernel_width_;
    int calculate_point_plateau_kernel_width_;

public:
    Preprocessing() : rclcpp::Node("preprocessing")
    {
        declare_parameter("filter_close_points_distance_m", 0.5);
        get_parameter("filter_close_points_distance_m", filter_close_points_distance_m_);

        declare_parameter("filter_max_intensity", 256.0);
        get_parameter("filter_max_intensity", filter_max_intensity_);

        declare_parameter("use_gyroscopic_undistortion", false);
        get_parameter("use_gyroscopic_undistortion", use_gyroscopic_undistortion_);

        declare_parameter("pre_downsample_leaf_size", 0.0);
        get_parameter("pre_downsample_leaf_size", pre_downsample_leaf_size_);

        declare_parameter("pc_normal_search_radius", 0.0);
        get_parameter("pc_normal_search_radius", pc_normal_search_radius_);

        declare_parameter("pc_normal_knn_points", 20);
        get_parameter("pc_normal_knn_points", pc_normal_knn_points_);

        declare_parameter("edge_threshold", 0.5);
        get_parameter("edge_threshold", edge_threshold_);

        declare_parameter("surfaces_threshold", 0.01);
        get_parameter("surfaces_threshold", surfaces_threshold_);

        declare_parameter("remove_statistical_outliers_knn_points", 10);
        get_parameter("remove_statistical_outliers_knn_points", remove_statistical_outliers_knn_points_);

        declare_parameter("remove_statistical_outliers_std_mul", 1.5);
        get_parameter("remove_statistical_outliers_std_mul", remove_statistical_outliers_std_mul_);

        declare_parameter("remove_transition_outliers_cos_angle_threshold", 0.9999);
        get_parameter("remove_transition_outliers_cos_angle_threshold", remove_transition_outliers_cos_angle_threshold_);

        declare_parameter("normalize_intensities_reference_range", 0.0);
        get_parameter("normalize_intensities_reference_range", normalize_intensities_reference_range_);

        declare_parameter("calculate_point_curvature_kernel_width", 0);
        get_parameter("calculate_point_curvature_kernel_width", calculate_point_curvature_kernel_width_);

        declare_parameter("calculate_point_plateau_kernel_width", 0);
        get_parameter("calculate_point_plateau_kernel_width", calculate_point_plateau_kernel_width_);

        declare_parameter("lidar_source_topic", "/livox");
        get_parameter("lidar_source_topic", lidar_source_topic_);

        declare_parameter("start_idx", 0); 
        get_parameter("start_idx", start_idx_);

        // declare_parameter("end_idx", std::numeric_limits<int>::max ()); 
        declare_parameter("end_idx", 0); // 0 means endless 
        get_parameter("end_idx", end_idx_);



        RCLCPP_INFO(get_logger(), "\033[1;32m---->\033[0m Preprocessing Started.");
        if (pre_downsample_leaf_size_ > 0.0){
            RCLCPP_INFO(get_logger(), "OBS clouds are downsampled in preprocessing, leafsize is: %f", pre_downsample_leaf_size_);
        }
        // if (!getParameter("/preprocessing/surf_thres", surf_thres))
        // {
        //     ROS_WARN("surf_thres not set, use default value: 0.2");
        //     surf_thres = 0.2;
        // }

        // if (!getParameter("/preprocessing/edge_thres", edge_thres))
        // {
        //     ROS_WARN("edge_thres not set, use default value: 4.0");
        //     edge_thres = 4.0;
        // }

        // if (!getParameter("/common/frame_id", frame_id))
        // {
        //     ROS_WARN("frame_id not set, use default value: lili_om");
        //     frame_id = "lili_om";
        // }

        subscriber_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);                                                            // create subscriber callback group
        rclcpp::SubscriptionOptions options;                                                                                                                         // create subscriver options
        options.callback_group = subscriber_cb_group_;                                                                                                               // add callbackgroup to subscriber options
        string lidar_topic = lidar_source_topic_ + "/lidar";
        sub_Lidar_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic, 100, std::bind(&Preprocessing::cloudHandler, this, _1), options); // add subscriber options to the subsriber constructor call..
        // sub_imu = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data_raw", 100, std::bind(&Preprocessing::imuHandler, this, _1), options);                  // make separate subscribe callback group?

        timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        process_timer = this->create_wall_timer(1ms, std::bind(&Preprocessing::processNext, this), timer_cb_group_);

        // sub_imu = nh.subscribe<sensor_msgs::Imu>("/livox/imu", 200, &Preprocessing::imuHandler, this);

        // pub_surf = this->create_publisher<sensor_msgs::msg::PointCloud2>("/surf_features", 100);
        // pub_edge = this->create_publisher<sensor_msgs::msg::PointCloud2>("/edge_features", 100);
        pub_cutted_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/preprocessed_point_cloud", 100);

        down_size_filter.setLeafSize(pre_downsample_leaf_size_, pre_downsample_leaf_size_, pre_downsample_leaf_size_);
    }
    ~Preprocessing(){}




    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_cloud_msg)
    {
        // cache point cloud in buffer
        if (first_lidar)
        {
            first_lidar = false; // used by imu_handler to see when the first lidar msg has arrived
        }

        cloud_queue.push_back(*lidar_cloud_msg);
    }


    // function that can remove points that are too close to the scanner i.e. auto-scans, weird function name will have to change it when I now how far i goes
    template <typename PointT>
    void filterProximateAndHighIntensePoints(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, float distance_threshold, float intensity_threshold)
    {

        // if the cloud are not the same, make sure head and size are the same
        if (&cloud_in != &cloud_out)
        {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }

        float squared_dist_threshold = distance_threshold * distance_threshold;
        size_t j = 0;
        for (size_t i = 0; i < cloud_in.points.size(); ++i)
        {
            if (cloud_in.points[i].x * cloud_in.points[i].x +
                cloud_in.points[i].y * cloud_in.points[i].y +
                cloud_in.points[i].z * cloud_in.points[i].z 
                < squared_dist_threshold  
                || cloud_in.points[i].intensity > intensity_threshold
                || (!pcl::isFinite(cloud_in.points[i]) || !pcl::isFinite(cloud_in.points[i]))) // calculating the squared distance of a point and comparing it to a threshold
                continue;
            
            // if (!pcl::isFinite(cloud_in.points[i]) || !pcl::isFinite(cloud_in.points[i]))
            // {
            //     continue;
            // }
            cloud_out.points[j] = cloud_in.points[i]; // if they are beyond the threshold assign the point to cloud out
            j++;                                      // count exchanged points
        }
        if (j != cloud_in.points.size())
        {
            cloud_out.points.resize(j);
        }

        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
    }

    template <typename PointT>
    double getDepth(PointT pt)
    {
        return sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    }

    template <typename PointT>
    double getSqDepth(PointT pt)
    {
        return pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
    }

    template <typename PointT>
    Eigen::Vector3d getNormalizedPositionVector(PointT pt)
    {
        Eigen::Vector3d position_vector = getPositionVector(pt).normalized();
        return position_vector;
    }

    template <typename PointT>
    Eigen::Vector3d getPositionVector(PointT pt)
    {
        Eigen::Vector3d position_vector(pt.x, pt.y, pt.z);
        return position_vector;
    }

    template <typename PointT>
    Eigen::Vector3d getXNormalizedVector(PointT pt)
    {
        Eigen::Vector3d position_vector(1.0, pt.y/pt.x, pt.z/pt.x);
        return position_vector;
    }

    // template <typename PointT>
    // Eigen::Vector3d getDepthNormalizedVector(PointT pt)
    // {
    //     double depth = getDepth(pt);
    //     Eigen::Vector3d position_vector(pt.x/depth, pt.y/depth, pt.z/depth);
    //     // position_vector.normalize();
    //     return position_vector;
    // }

    template <typename PointT>
    Eigen::Vector3d getSurfaceNormal(PointT pt)
    {
        Eigen::Vector3d normal_vector(pt.normal_x, pt.normal_y, pt.normal_z);
        return normal_vector;
    }



    template <typename PointT>
    void calculatePointNormals(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_in, pcl::PointCloud<PointT> &cloud_out)
    {
        // Create the normal estimation class, and pass the input dataset to it
        // pcl::NormalEstimationOMP<PointType, PointType> normal_estimator;
        // pcl::NormalEstimationOMP<PointTypeWTime, PointTypeWTime> normal_estimator;
        pcl::NormalEstimationOMP<PointT, PointT> normal_estimator;
        normal_estimator.useSensorOriginAsViewPoint();
        normal_estimator.setInputCloud(cloud_in);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>()); // boost shared ptr?
        // pcl::search::KdTree<PointTypeWTime>::Ptr tree(new pcl::search::KdTree<PointTypeWTime>()); // boost shared ptr?
        // pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>()); // boost shared ptr?
        normal_estimator.setSearchMethod(tree);

        // // Output datasets
        // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

        if (pc_normal_search_radius_ > 0.0){
            normal_estimator.setRadiusSearch(pc_normal_search_radius_); // Use all neighbors in a sphere of radius x meters
        }
        else{
            normal_estimator.setKSearch(pc_normal_knn_points_); // use x nearest points, more robust to cloud scale variation
        }

        // Compute the features
        normal_estimator.compute(cloud_out);

        // cloud_normals->size () should have the same size as the input cloud->size ()*
    }

    template <typename PointT>
    void removeStatisticalOutliers(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_in, pcl::PointCloud<PointT> &cloud_out)
    {
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(cloud_in);
        sor.setMeanK(remove_statistical_outliers_knn_points_);        // n amount of points to use in mean distance estimation
        sor.setStddevMulThresh(remove_statistical_outliers_std_mul_); // x*std outlier rejection threshold - 2.0 should cover 98% percent of gaussin dist points
        sor.filter(cloud_out);
    }

    template <typename PointT>
    void removeTransitionOutliers(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out)
    {
        // PointType point;
        size_t size_in = cloud_in.points.size();
        cloud_out.points[0] = cloud_in.points[0];
        size_t j = 1;
        // size_t section_start = 1;
        // size_t section_end = 0;
        // double nominal_step_length = 10.0;
        for (size_t i = 1; i < size_in-1; ++i)
        {
            Eigen::Vector3d shooting_vector_fore = getPositionVector(cloud_in.points[i-1]); // P
            Eigen::Vector3d shooting_vector_query = getPositionVector(cloud_in.points[i]); // Q
            Eigen::Vector3d shooting_vector_aft = getPositionVector(cloud_in.points[i+1]); 

            // Eigen::Vector3d step_vector_to = (shooting_vector_query - shooting_vector_fore ); //
            // Eigen::Vector3d step_vector_from = (shooting_vector_aft - shooting_vector_query ); // 
            Eigen::Vector3d step_vector_across = (shooting_vector_fore - shooting_vector_aft ); // 
            // Eigen::Vector3d center_vector(1.0,0.0,0.0);
            // double step_length = (step_vector_to.norm()*step_vector_to.norm() + step_vector_from.norm()*step_vector_from.norm());
            // double step_length = (step_vector_to.norm() + step_vector_from.norm()) * (step_vector_to.norm() + step_vector_from.norm());
            // double step_length = step_vector_to.norm();
            // double depth = getDepth(cloud_in.points[i]);
            // double cos_angle = abs(step_vector_to.normalized().dot(shooting_vector_fore.normalized()));
            // double cos_angle_from_center = abs(center_vector.dot(shooting_vector_query.normalized()));
            // double scan_wander = (shooting_vector_fore.normalized() - shooting_vector_aft.normalized()).norm()*depth;
            // scan_wander *= scan_wander;
            // double distance_to_shooting_vector = step_vector_to.cross(shooting_vector_fore).norm() / shooting_vector_fore.norm();
            
            double intensity = (double)floor(cloud_in.points[i].intensity);



            double cos_angle = abs(step_vector_across.normalized().dot(shooting_vector_query.normalized()));
            // if (distance_to_shooting_vector < remove_transition_outliers_cos_angle_threshold_ && cos_angle >= 0.9986){ 
            // if (step_length * (1.0/ (cos_angle_from_center*cos_angle_from_center))/(scan_wander) > remove_transition_outliers_cos_angle_threshold_  ) {//&& cos_angle >= 0.9986){ 
            bool incidence = cos_angle > remove_transition_outliers_cos_angle_threshold_ ;
            // bool hidden = step_length >= 0.1 * shooting_vector_query.norm() && shooting_vector_query.norm() > shooting_vector_fore.norm();
            bool hidden = false;
            bool intensity_thresh = intensity < 0.0  || intensity > 260.0;
            if (incidence || hidden || intensity_thresh){ 
            // if (distance_to_shooting_vector < remove_transition_outliers_cos_angle_threshold_){ 
                // RCLCPP_INFO(get_logger(), "point %i removed cos angle = %f", i, cos_angle);
                // RCLCPP_INFO(get_logger(), "scan_wander %f", scan_wander);
                // RCLCPP_INFO(get_logger(), "scan_wander/depth %f", scan_wander/depth);
                // RCLCPP_INFO(get_logger(), "distance_to_shooting_vector %f", distance_to_shooting_vector);
                // RCLCPP_INFO(get_logger(), "step_length %f", step_length);
                // RCLCPP_INFO(get_logger(), "step_length/depth %f", step_length/depth);
                // RCLCPP_INFO(get_logger(), "threshold %f", step_length/(scan_wander) /depth);
                continue;
            }
            // section_start = i;
            // RCLCPP_INFO(get_logger(), "point %i added %i cos angle = %f", i, j,cos_angle);
            cloud_out.points[j] = cloud_in.points[i]; 
            j++;                                      // count exchanged points
        }

        if (j != cloud_in.points.size())
        {
            cloud_out.points.resize(j);
        }

        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
        // RCLCPP_INFO(get_logger(), "points removed %i ", size_in - cloud_out.points.size());
    }
    

    template <typename PointT>
    void assignEdgeAndSurface(boost::shared_ptr<pcl::PointCloud<PointT>> &edges, boost::shared_ptr<pcl::PointCloud<PointT>> &surfs)
    {
        size_t cloud_size = lidar_cloud_in->points.size();
        // this is in meters when using plateau
        //  double edge_threshold_ = 0.5; // edge treshold, in meters when using plateau
        //  double surfaces_threshold_ = 0.01; // surfaces threshold

        PointType point;
        for (size_t i = 0; i < cloud_size; i++)
        {
            point = lidar_cloud_in->points[i];

            if (point.curvature > surfaces_threshold_ && point.curvature < edge_threshold_)
            {
                continue;
            }
            else if (point.curvature > edge_threshold_)
            {
                edges->points.push_back(point);
                // RCLCPP_INFO(get_logger(), "Point should be added to edge");
            }
            else if (point.curvature < surfaces_threshold_)
            {
                surfs->points.push_back(point);
                // RCLCPP_INFO(get_logger(), "Point should be added to surf");
            }
        }
    }

    template <typename PointT>
    void normalizeIntensities(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out)
    {
        double range_reference = normalize_intensities_reference_range_; // could be actively determined from cloudscale..
        double intensity_normalized = 0.0;
        Eigen::Vector3d range_vector(0.0, 0.0, 0.0);
        Eigen::Vector3d normal_vector(0.0, 0.0, 0.0);
        // double max_intesity = std::numeric_limits<double>::min ();
        for (size_t i = 0; i < cloud_in.points.size(); ++i)
        {
            double range = getDepth(cloud_in.points[i]);
            normal_vector = getSurfaceNormal(cloud_in.points[i]);
            range_vector = - getNormalizedPositionVector(cloud_in.points[i]);
            // double cos_incidence_angle = abs(range_vector.transpose() * normal_vector);
            double cos_incidence_angle = range_vector.dot(normal_vector); // both are unit vector therefore this should be the cosine of the angle between
            double intensity = (double)floor(cloud_in.points[i].intensity);
            double timestamp = (double)cloud_in.points[i].intensity - intensity; // save the timestamp of the point
            // intensity_normalized = intensity * (range / range_reference) * (range / range_reference) * (1.0 / cos_incidence_angle);
            // intensity_normalized = intensity * pow((range / range_reference), 2.3) * (1.0 / cos_incidence_angle);
            // intensity_normalized = intensity * pow((range / range_reference), 0.5) * pow(cos_incidence_angle, -0.3);
            
            if (range_reference > 0.0)  {
                intensity_normalized = intensity * pow((range / range_reference), 0.5) *  pow(cos_incidence_angle, -0.3);
            } else if (range_reference == 0.0){
                intensity_normalized = intensity * pow(cos_incidence_angle, -0.3);
            }
            
            // intensity_normalized = intensity * (range / range_reference) * (1.0 / cos_incidence_angle);
            // intensity_normalized = intensity * (1.0 / cos_incidence_angle);

            // if (intensity_normalized > filter_max_intensity_)
            // {
            //     intensity_normalized = filter_max_intensity_;
            // }
            
            intensity_normalized = round(intensity_normalized);
            cloud_out.points[i].intensity = intensity_normalized + timestamp; // re add the timestamp with the new intensity value

            // if (intensity_normalized > max_intesity) {
            //     max_intesity = intensity_normalized;
            // }

        }

        // for (size_t i = 0; i < cloud_in.points.size(); ++i) {
        //     cloud_out.points[i].intensity = (int)round(cloud_out.points[i].intensity/ max_intesity * 255.0);
        // }
    }



    template <typename PointT>
    void embedPointTime(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_in, boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_out, const double scan_dt){
        // double scan_time = scan_dt;
        // double scan_time = 0.1;
        double point_dt = scan_dt / (cloud_in->points.size() - 1);
        
        for (size_t i=0; i < cloud_in->points.size(); i++){

            // double dt_i = point_dt * (i+1); // +1 because the last point in the previous scan is considered t=0 therefore first(zeroth) point in this scan is at t=1*dt
            // // this also makes sure that the full scan_time is imbedded in the final point, and can be retrieved for later use. 
            
            double dt_i = point_dt * i; 
            cloud_out->points[i].intensity += dt_i;
        }
    }



    double validateScanTime(double new_dt)
    {
        if (scan_dts.size() < 30){
            scan_dts.push_front(new_dt);
            // RCLCPP_INFO(get_logger(), "scan time %f", new_dt);
            return new_dt;
        }

        size_t n = scan_dts.size();
        double sum = 0.0;
        double sum_sq = 0.0;
        for (size_t i=0; i < n; i++){
            sum += scan_dts[i];
            sum_sq += scan_dts[i]*scan_dts[i];
        }
        double mean_dt = sum / (double)n;

        // double std_dt = sqrt((sum_sq)/(double)n - mean_dt*mean_dt );
        double std_dt = sqrt( ((sum_sq) - sum*sum/(double)n) / (double)(n-1)  );



        if (abs(new_dt - mean_dt) > 10*std_dt){
            RCLCPP_WARN(get_logger(), "Bad scan dt detected!! %f,  using mean instead %f. Std is: %f", new_dt, mean_dt, std_dt );
            return mean_dt;
        }
        // RCLCPP_INFO(get_logger(), "scan time %f,  mean %f, std %f.", new_dt, mean_dt, std_dt);

        scan_dts.pop_back();
        scan_dts.push_front(new_dt);

        return new_dt;
    }

    void processNext()
    {
        rclcpp::Clock system_clock;
        // get next frame in buffer..
        if (cloud_queue.size() <= 1) // leave atleast 1, to get the time of the next cloud.. (or set next time to + 0.1 (scanning time) staticly)
        {
            return;
        }
        else
        {
            time_scan = toSec(cloud_queue.front().header.stamp);
            current_cloud_msg = cloud_queue.front(); // puts the next cloud to variable current
            cloud_queue.pop_front();                 // removes element from the queue

            // set header id and timestamp
            cloud_header = current_cloud_msg.header;
            cloud_header.frame_id = frame_id;

            time_scan_next = toSec(cloud_queue.front().header.stamp); // why take time from the next? and should it be double?
            // !answer: the start-time of the next scan is the end of the current, this is needed to integrate the imu, and thats why the buffer needs to leave 1 frame

            if (cloud_queue.size() > 5)
            {
                std::string msg = "Warning: preprocessing buffer size is:" + std::to_string(cloud_queue.size()) +", previous frame process time was: " + std::to_string(frame_procces_time*1000) + "ms";
                RCLCPP_WARN_THROTTLE(get_logger(),* get_clock(), 2000, msg);
                // RCLCPP_WARN(get_logger(), "Warning: preprocessing buffer size is: %i, previous frame process time was: %fms", cloud_queue.size(), frame_procces_time*1000);
                // RCLCPP_WARN(get_logger(), "Buffer size is: %i", cloud_queue.size());
            }
        }
        // RCLCPP_INFO(get_logger(), "Previous frame process time was: %fs", frame_procces_time);

        rclcpp::Time time_preproc_start = system_clock.now();

        latest_frame_idx++;
        if (latest_frame_idx < start_idx_ || (latest_frame_idx > end_idx_ && end_idx_ > 0) ){
            RCLCPP_INFO(get_logger(), "Skipping frames %i", latest_frame_idx);
            return;
        }

        // #ifndef __INTELLISENSE__ // to ignore error from missing overload of function, should still work on the new sensor_msg::msg::PointCloud2
        // pcl::fromROSMsg(current_cloud_msg, *lidar_cloud_in_no_normals);
        // // pcl::fromROSMsg(current_cloud_msg, *lidar_cloud_in_with_time);
        // #endif
        fromROSMsg(current_cloud_msg, *lidar_cloud_in_no_normals );




        double scan_dt = time_scan_next - time_scan;
        scan_dt = validateScanTime(scan_dt); // safeguard againt hitches/pauses in the recording
        // embedPointTime(lidar_cloud_in_with_time, lidar_cloud_in_with_time, scan_dt);
        // embedPointTime(lidar_cloud_in, lidar_cloud_in, scan_dt);
        embedPointTime(lidar_cloud_in_no_normals, lidar_cloud_in_no_normals, scan_dt);

        // ESSENTIAL PREPROCESSING
        filterProximateAndHighIntensePoints(*lidar_cloud_in_no_normals, *lidar_cloud_in_no_normals, filter_close_points_distance_m_, filter_max_intensity_); // removes invalid points within a distance of x m from the center of the lidar
        // removeTransitionOutliers(*lidar_cloud_in_no_normals, *lidar_cloud_in_no_normals);
        // removeStatisticalOutliers(lidar_cloud_in_no_normals, *lidar_cloud_in_no_normals);
        pcl::copyPointCloud(*lidar_cloud_in_no_normals, *lidar_cloud_in); // change pointtype to point cloud with normal data

        // double scan_dt = time_scan_next - time_scan;
        // scan_dt = validateScanTime(scan_dt); // safeguard againt hitches/pauses in the recording
        // // embedPointTime(lidar_cloud_in_with_time, lidar_cloud_in_with_time, scan_dt);
        // embedPointTime(lidar_cloud_in, lidar_cloud_in, scan_dt);

        // filterProximateAndHighIntensePoints(*lidar_cloud_in_with_time, *lidar_cloud_in_with_time, filter_close_points_distance_m_, 200.0); // removes invalid points within a distance of x m from the center of the lidar
        // removeStatisticalOutliers(lidar_cloud_in_with_time, *lidar_cloud_in_with_time);
        // pcl::copyPointCloud(*lidar_cloud_in_with_time, *lidar_cloud_in); // change pointtype to point cloud with normal data

        if (pc_normal_knn_points_ > 0 || pc_normal_search_radius_ > 0.0){
            calculatePointNormals(lidar_cloud_in, *lidar_cloud_in); // openMP multi-processing accelerated
        }

        // NON-ESSENTIAL PREPROCESSING.
        
        // if (normalize_intensities_reference_range_ > 0.0){
        //     normalizeIntensities(*lidar_cloud_in, *lidar_cloud_in);
        // }


        sensor_msgs::msg::PointCloud2 cloud_cutted_msg;

        toROSMsg(*lidar_cloud_in, cloud_cutted_msg);

        cloud_cutted_msg.header.stamp = cloud_header.stamp;
        cloud_cutted_msg.header.frame_id = frame_id;
        pub_cutted_cloud->publish(cloud_cutted_msg);

        rclcpp::Time time_preproc_end = system_clock.now();

        frame_procces_time = time_preproc_end.seconds() - time_preproc_start.seconds();

    }
};

int main(int argc, char **argv)
{
    // --- multi process --- 
    rclcpp::init(argc, argv);
    auto processing_node = std::make_shared<Preprocessing>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(processing_node);

    executor.spin();
    rclcpp::shutdown();
    return 0;

    // --- single process --- 
    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<Preprocessing>());
    // rclcpp::shutdown();
    // return 0;
}