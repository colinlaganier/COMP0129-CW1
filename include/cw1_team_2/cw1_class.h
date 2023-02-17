/**
  **********************************************************************************
  * @file     cw1_class.h
  * @author   Colin Laganier, Jacob Nash, Carl Parsons
  * @date     2023-02-15
  * @brief   This file contains the header information for the cw1 class.
  *          The goal of the class is to enable the robot to perform the tasks 
  *          in ROS.
  **********************************************************************************
  * @attention  Requires ros, moveit, pcl, tf2 to be installed.
  */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW1_CLASS_H_
#define CW1_CLASS_H_

// system includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
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

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
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

// TF specific includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// include services from the spawner package
#include "cw1_world_spawner/Task1Service.h"
#include "cw1_world_spawner/Task2Service.h"
#include "cw1_world_spawner/Task3Service.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

class cw1
{
public:
  /* ----- class member variables ----- */

  /** \brief Define some useful constant values */
  std::string base_frame_ = "panda_link0";
  double gripper_open_ = 80e-3;
  double gripper_closed_ = 0.0;

  /** \brief Parameters for Task 1 */
  double z_offset_ = 0.125;
  double angle_offset_ = 3.14159 / 4.0;
  double approach_distance_ = 0.25;  

  /** \brief Parameters for Task 2 */
  std::vector<unsigned char, std::allocator<unsigned char> >colour_image_data;
  ros::Subscriber colour_image_sub_;
  int colour_image_width_;
  int colour_image_height_;
  int colour_image_midpoint_;
  int color_channels_ = 3;

  /** \brief Parameters for Task 3 */
  enum Color {red, blue, purple, green, none};
  Color Colors;

  struct TargetBasket {
    geometry_msgs::Point coordinates;
    // Initialising distance to 100m to ensure that the first basket is always the closest
    float distance_to_cube = 100;
    bool is_empty = true;
  };

  float basket_height_;
  float cube_height_;
  float camera_offset_;
  int cube_basket_cutoff_;
  double position_precision_;
  geometry_msgs::Point scan_position_;

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


  ////////////////////////////////////////////////////////////////////////////////
  // Class member functions
  ////////////////////////////////////////////////////////////////////////////////

  // constructor
  cw1(ros::NodeHandle nh);

  /** \brief Function to load pre-defined constants values */
  void
  load_config();

  /** \brief  service callbacks for Tasks 1, 2, and 3 */
  bool 
  t1_callback(cw1_world_spawner::Task1Service::Request &request,
    cw1_world_spawner::Task1Service::Response &response);

  bool
  t2_callback(cw1_world_spawner::Task2Service::Request &request,
    cw1_world_spawner::Task2Service::Response &response);

  bool 
  t3_callback(cw1_world_spawner::Task3Service::Request &request,
    cw1_world_spawner::Task3Service::Response &response);

  /** \brief Point cloud data callback function */
  void   
  pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg);
  
  /** \brief RGB camera image callback function*/
  void 
  colourImageCallback(const sensor_msgs::Image& msg);


  ////////////////////////////////////////////////////////////////////////////////
  // Task 1 functions
  ////////////////////////////////////////////////////////////////////////////////

  /** \brief Task 1 function, pick and place an object from a target
   * to a goal using the gripper. 
   *
   * \input[in] object_loc location of object to pick cube
   * \input[in] goal_loc location to place cube in basket
   *
   * \return true if object is picked and placed
  */
  bool
  pickPlace(geometry_msgs::Point object, geometry_msgs::Point goal);

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
  

  /** \brief Function to publish the filtered point cloud message.
    *
    * \input[in] pc_pub ROS publisher
    * \input[in] pc point cloud to publish
    */
  void 
  pubFilteredPCMsg(ros::Publisher & pc_pub, PointC & pc);

  /** \brief Function to apply a Pass Through filter to the point cloud.
    *
    * \input[in] in_cloud_ptr input point cloud
    * \input[in] out_cloud_ptr output point cloud
    */
  void 
  applyPT(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);

  ////////////////////////////////////////////////////////////////////////////////
  // Task 2 functions
  ////////////////////////////////////////////////////////////////////////////////

  /** \brief Task 2 function, survey the area for baskets and cubes.
    *
    * \input[in] basket_locs vector of basket positions
    * 
    * \return vector of basket colours in the order they are found
    */
  std::vector<std::string>
  task_2(std::vector<geometry_msgs::PointStamped> basket_locs);
  
  /** \brief Function to convert a geometry_msgs point to a pose.
    *
    * \input[in] point point to convert to pose
    *
    * \return pose of the point
    */
  geometry_msgs::Pose
  point2Pose(geometry_msgs::Point point);

  /** \brief Function to identify the colour of the basket based on
    * the current camera feed and a given position.
    *
    * \input[in] basket_loc location of the basket of interest
    *
    * \return colour of the cube
    */
  std::string
  survey(geometry_msgs::Point basket_loc);

  ////////////////////////////////////////////////////////////////////////////////
  // Task 3 functions
  ////////////////////////////////////////////////////////////////////////////////

  /** \brief Task 3 function, identify the cubes and backets in the scene
    * and place the cube in the correct basket based on their colour and baskets. 
    * 
    * \return true if every possible cube is placed in a basket
    */
  bool 
  task_3();

  /** \brief Function to identify the colour of the cube based on
    *   the RGB value of a point in the point cloud.  
    *
    * \input[in] rgb RGB value of the point
    *
    * \return Colour of the cube
    */
  Color 
  identify_color(uint32_t rgb);

    /** \brief Function to cluster the group of point clouds into
    *  individual point clouds.
    *
    * \input[in] cloud filtered point cloud to cluster
    *
    * \return vector of individual point clouds
    */
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> 
  cluster_pointclouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

  /** \brief Function to identify a basket that matches the cube.
    *
    * \input[in] cube tuple of cube location and colour
    * \input[in] basket_data vector of basket locations and colours
    *
    * \return a TargetBasket struct containing the basket location and colour
    */
  cw1::TargetBasket 
  identify_basket(std::tuple<geometry_msgs::Point, Color> cube, 
    std::vector<std::tuple<geometry_msgs::Point, Color>> &basket_data);

  
protected:
  /** \brief Debug mode. */
  bool debug_;
};

#endif // end of include guard for CW1_CLASS_H_
