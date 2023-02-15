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
  bool 
  task_1(geometry_msgs::PoseStamped object_loc, geometry_msgs::PointStamped goal_loc);

  std::vector<std::string>
  task_2(std::vector<geometry_msgs::PointStamped> basket_locs);
  
  bool task_3();

  std::string 
  identify_basket();

void cloudCallBackOne(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg);

  void pubFilteredPCMsg(ros::Publisher & pc_pub, PointC & pc);
  
  // void separateKMeans(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);

  bool passThroughCallback(cw1_team_2::pass_through::Request &request, cw1_team_2::pass_through::Response &response);
  bool setArmCallback(cw1_team_2::set_arm::Request &request, cw1_team_2::set_arm::Response &response);


  void computeCubeCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Vector4f &centroid);

  void separateKMeans(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters);



  void applyPT(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);

  // void imageCallback(const sensor_msgs::ImageConstPtr& msg);

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

 /** \brief service server for task 3 */
  ros::ServiceServer set_arm_srv_;
  ros::ServiceServer pass_through_srv_;

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
  
  /** \brief ROS geometry message point. */
  geometry_msgs::PointStamped g_cyl_pt_msg;
  
  /** \brief ROS pose publishers. */
  ros::Publisher g_pub_pose;
  
  /** \brief Voxel Grid filter's leaf size. */
  double g_vg_leaf_sz;
  
  /** \brief Point Cloud (input) pointer. */
  PointCPtr g_cloud_ptr;
  
  /** \brief Point Cloud (filtered) pointer. */
  PointCPtr g_cloud_filtered, g_cloud_filtered2;
  
  /** \brief Point Cloud (filtered) sensros_msg for publ. */
  sensor_msgs::PointCloud2 g_cloud_filtered_msg;
  
  /** \brief Point Cloud (input). */
  pcl::PCLPointCloud2 g_pcl_pc;
  
  /** \brief Voxel Grid filter. */
  pcl::VoxelGrid<PointT> g_vx;
  
  /** \brief Pass Through filter. */
  pcl::PassThrough<PointT> g_pt;
  
  /** \brief Pass Through min and max threshold sizes. */
  double g_pt_thrs_min, g_pt_thrs_max;
  
  /** \brief KDTree for nearest neighborhood search. */
  pcl::search::KdTree<PointT>::Ptr g_tree_ptr;
  
  /** \brief Normal estimation. */
  pcl::NormalEstimation<PointT, pcl::Normal> g_ne;
  
  /** \brief Cloud of normals. */
  pcl::PointCloud<pcl::Normal>::Ptr g_cloud_normals, g_cloud_normals2;
  
  /** \brief Nearest neighborhooh size for normal estimation. */
  double g_k_nn;
  
  /** \brief SAC segmentation. */
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> g_seg; 
  
  /** \brief Extract point cloud indices. */
  pcl::ExtractIndices<PointT> g_extract_pc;

  /** \brief Extract point cloud normal indices. */
  pcl::ExtractIndices<pcl::Normal> g_extract_normals;
  
  /** \brief Point indices for plane. */
  pcl::PointIndices::Ptr g_inliers_plane;
    
  /** \brief Point indices for cylinder. */
  pcl::PointIndices::Ptr g_inliers_cylinder;
  
  /** \brief Model coefficients for the plane segmentation. */
  pcl::ModelCoefficients::Ptr g_coeff_plane;
  
  /** \brief Model coefficients for the culinder segmentation. */
  pcl::ModelCoefficients::Ptr g_coeff_cylinder;
  
  /** \brief Point cloud to hold plane and cylinder points. */
  PointCPtr g_cloud_plane, g_cloud_cylinder;
  
  /** \brief cw1Q1: TF listener definition. */
  tf::TransformListener g_listener_;

  /** \brief Task 3 variables. */
  enum Color {red, blue, purple, green, none};

  std::vector<std::tuple<geometry_msgs::Point, Color>> cube_centroids;
  std::vector<std::tuple<geometry_msgs::Point, Color>> basket_centroids;

  double offset_x, offset_y;
  
protected:
  /** \brief Debug mode. */
  bool debug_;
};

#endif // end of include guard for CW1_CLASS_H_
