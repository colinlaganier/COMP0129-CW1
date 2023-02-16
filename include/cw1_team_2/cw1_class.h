/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW1_CLASS_H_
#define CW1_CLASS_H_

// system includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
// #include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/Image.h>


// Config includes
#include <iostream>
#include <fstream>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/point_cloud.h>

#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>

// OpenCV includes
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/opencv.hpp>

// TF specific includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;


// include services from the spawner package - we will be responding to these
#include "cw1_world_spawner/Task1Service.h"
#include "cw1_world_spawner/Task2Service.h"
#include "cw1_world_spawner/Task3Service.h"

// // include any services created in this package
#include "cw1_team_2/set_arm.h"
#include "cw1_team_2/pass_through.h"


class cw1
{
public:

  /* ----- class member functions ----- */

  // constructor
  cw1(ros::NodeHandle nh);

  // service callbacks for tasks 1, 2, and 3
  bool 
  t1_callback(cw1_world_spawner::Task1Service::Request &request,
    cw1_world_spawner::Task1Service::Response &response);
  // std::vector<std::string>  
  bool
  t2_callback(cw1_world_spawner::Task2Service::Request &request,
    cw1_world_spawner::Task2Service::Response &response);
  bool 
  t3_callback(cw1_world_spawner::Task3Service::Request &request,
    cw1_world_spawner::Task3Service::Response &response);


  /** \brief MoveIt function for moving the move_group to the target position.
    *
    * \input[in] target_pose pose to move the arm to
    *
    * \return true if moved to target position 
    */
  bool 
  moveArm(geometry_msgs::Pose target_pose);

  /** \brief MoveIt function for moving the gripper fingers to a new position. 
    *
    * \input[in] width desired gripper finger width
    *
    * \return true if gripper fingers are moved to the new position
    */
  bool 
  moveGripper(float width);
  
  /** \brief Task 1 function, pick and place an object from a target
   * to a goal using the gripper. 
    *
    * \input[in] object_loc location of object to pick
    * \input[in] goal_loc location of basket to place object
    *
    * \return true if pick and place operation is succesful
    */


  std::vector<std::string>
  task_2(std::vector<geometry_msgs::PointStamped> basket_locs);
  
  bool task_3();

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg);

  void pubFilteredPCMsg(ros::Publisher & pc_pub, PointC & pc);

  void applyPT(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);

  /* ----- class member variables ----- */

  /** \brief Define some useful constant values */
  std::string base_frame_ = "panda_link0";
  double gripper_open_ = 80e-3;
  double gripper_closed_ = 0.0;

  /** \brief Parameters for Task 1 (pick and place). */
  double z_offset_ = 0.125;
  double angle_offset_ = 3.14159 / 4.0;
  double approach_distance_ = 0.25;

  /** \brief Node handle. */
  ros::NodeHandle nh_;

  /** \brief  service servers for advertising ROS services  */
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  /** \brief MoveIt interface to move groups to seperate the arm and the gripper,
  * these are defined in urdf. */
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

  /** \brief MoveIt interface to interact with the moveit planning scene 
    * (eg collision objects). */
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  /** \brief Node handle. */
  ros::NodeHandle g_nh;
  
  /** \brief The input point cloud frame id. */
  std::string g_input_pc_frame_id_;

  /** \brief ROS publishers. */
  ros::Publisher g_pub_cloud;

  /** \brief ROS pose publishers. */
  ros::Publisher g_pub_pose;
  
  /** \brief Point Cloud (input) pointer. */
  PointCPtr g_cloud_ptr;
  
  /** \brief Point Cloud (filtered) pointer. */
  PointCPtr g_cloud_filtered;

  /** \brief Point Cloud (input). */
  pcl::PCLPointCloud2 g_pcl_pc;
  
  /** \brief Point Cloud (filtered) sensros_msg for publ. */
  sensor_msgs::PointCloud2 g_cloud_filtered_msg;
  
  /** \brief Pass Through filter. */
  pcl::PassThrough<PointT> g_pt;
  
  /** \brief Pass Through min and max threshold sizes. */
  double g_pt_thrs_min, g_pt_thrs_max;


  /** \brief Task 2 functions*/
  geometry_msgs::Pose
  point2Pose(geometry_msgs::Point point);
  bool
  pickPlace(geometry_msgs::Point object, geometry_msgs::Point goal);

  std::string
  survey(geometry_msgs::Point basket_loc);
  void colourImageCallback(const sensor_msgs::Image& msg);

  std::vector<unsigned char, std::allocator<unsigned char> >colour_image_data;
  ros::Subscriber colour_image_sub_;
  int colour_image_width_;
  int colour_image_height_;
  int colour_image_middle_;
  int color_channels_ = 3;

  int image_width_ = 640;
  int image_height_ = 480;
  // int middle_index_ = color_channels_ * ((image_width_ * (image_height_ / 2 - 1)) + (image_width_ / 2));
  // cw1::color_channels_ * ((cw1::colour_image_width_ * 
  //         (cw1::colour_image_height_ / 2 - 1)) + (cw1::colour_image_width_ / 2));
  // 461757

  /** \brief Task 3 variables. */
  enum Color {red, blue, purple, green, none};
  Color Colors;
  Color identify_color(uint32_t rgb);

  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cluster_pointclouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);


  float basket_height_ = 0.40;
  float cube_height_ = 0.15;
  float camera_offset_ = 0.0425;
  int cube_basket_cutoff_ = 1000;
  double position_precision_ = 1000.0;


  geometry_msgs::Point scan_position_;

  struct TargetBasket {
    geometry_msgs::Point coordinates;
    // Initialising distance to 100m to ensure that the first basket is always the closest
    float distance_to_cube = 100;
    bool is_empty = true;
  };

  cw1::TargetBasket identify_basket(std::tuple<geometry_msgs::Point, Color> cube, std::vector<std::tuple<geometry_msgs::Point, Color>> &basket_data);

  bool load_config();
  
protected:
  /** \brief Debug mode. */
  bool debug_;
};

#endif // end of include guard for CW1_CLASS_H_
