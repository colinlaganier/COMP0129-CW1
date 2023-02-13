/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

#include <cw1_team_2/cw1_class.h>
// #include "cw1_class.h"

// Not in use just added the PCL elements to check something 
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

///////////////////////////////////////////////////////////////////////////////

cw1::cw1(ros::NodeHandle nh):
  g_cloud_ptr (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
  g_cloud_filtered2 (new PointC), // filtered point cloud
  g_cloud_plane (new PointC), // plane point cloud
  g_cloud_cylinder (new PointC), // cylinder point cloud
  g_tree_ptr (new pcl::search::KdTree<PointT> ()), // KdTree
  g_cloud_normals (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_cloud_normals2 (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_inliers_plane (new pcl::PointIndices), // plane seg
  g_inliers_cylinder (new pcl::PointIndices), // cylidenr seg
  g_coeff_plane (new pcl::ModelCoefficients), // plane coeff
  g_coeff_cylinder (new pcl::ModelCoefficients), // cylinder coeff
  debug_ (false)
{
  /* class constructor */

  nh_ = nh;

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw1::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw1::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw1::t3_callback, this);

  // helper services 
  set_arm_srv_ = nh_.advertiseService("/set_arm",
  &cw1::setArmCallback, this);
  pass_through_srv_ = nh_.advertiseService("/pass_through",
  &cw1::passThroughCallback, this);

  // OpenCV test
  // cv::namedWindow("view");
  // image_transport::ImageTransport it(nh_);
  // image_transport::Subscriber sub = it.subscribe("camera/image", 1, &cw1::imageCallback, this);


  // Define the publishers
  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1, true);
  g_pub_pose = nh.advertise<geometry_msgs::PointStamped> ("cyld_pt", 1, true);
  
  // Define public variables
  g_vg_leaf_sz = 0.01; // VoxelGrid leaf size: Better in a config file
  g_pt_thrs_min = 0.0; // PassThrough min thres: Better in a config file
  g_pt_thrs_max = 0.805; // PassThrough max thres: Better in a config file
  g_k_nn = 50; // Normals nn size: Better in a config file

  ROS_INFO("cw1 class initialised");
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t1_callback(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */

  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  bool success = task_1(request.object_loc, request.goal_loc);

  // response.success = success;
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t2_callback(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  // geometry_msgs::PointStamped basket_locs[3] = {request.basket_locs};
  std::vector<std::string> colours = task_2(request.basket_locs);
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t3_callback(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  bool success = task_3();

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::moveArm(geometry_msgs::Pose target_pose)
{
  /* This function moves the move_group to the target position */

  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // google 'c++ conditional operator' to understand this line
  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  arm_group_.move();

  return success;
}

// bool
// cw1::moveArm(geometry_msgs::Pose target_pose)
// {
//   /* This function moves the move_group to the target position */

//   // setup the target pose
//   ROS_INFO("Setting pose target");
//   arm_group_.setPoseTarget(target_pose);

//   // create a movement plan for the arm
//   ROS_INFO("Attempting to plan the path");
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//   bool success = (arm_group_.plan(my_plan) ==
//     moveit::planning_interface::MoveItErrorCode::SUCCESS);

//   // google 'c++ conditional operator' to understand this line
//   ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

//   // execute the planned path
//   arm_group_.move();

//   return success;
// }
// [ INFO] [1676243570.290049001, 8.813000000]: Approach pose: 
// [ INFO] [1676243570.290066324, 8.813000000]: x: 0.349304
// [ INFO] [1676243570.290078276, 8.813000000]: y: -0.092831
// [ INFO] [1676243570.290089928, 8.813000000]: z: 0.404000

///////////////////////////////////////////////////////////////////////////////

bool
cw1::moveGripper(float width)
{
  /* this function moves the gripper fingers to a new position. Joints are:
      - panda_finger_joint1
      - panda_finger_joint2
  */

  // safety checks
  if (width > gripper_open_) width = gripper_open_;
  if (width < gripper_closed_) width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  hand_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

bool 
cw1::task_1(geometry_msgs::PoseStamped object_loc, geometry_msgs::PointStamped goal_loc)
{
  /* This function picks up an object using a pose and drop it at at goal pose */

  // define grasping as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);

  geometry_msgs::Pose grasp_pose = object_loc.pose;
  grasp_pose.orientation = grasp_orientation;
  grasp_pose.position.z += z_offset_;

  // set the desired pre-grasping pose
  geometry_msgs::Pose approach_pose;
  approach_pose = grasp_pose;
  approach_pose.position.z += approach_distance_;

  // Print the approach pose
  ROS_INFO("Approach pose: ");
  ROS_INFO("x: %f", approach_pose.position.x);
  ROS_INFO("y: %f", approach_pose.position.y);
  ROS_INFO("z: %f", approach_pose.position.z);


  geometry_msgs::Pose drop_pose;
  drop_pose = grasp_pose;
  drop_pose.position = goal_loc.point;
  drop_pose.position.z = 0.4; 

  bool success = true;

  ROS_INFO("Begining pick operation");

  // move the arm above the object
  success *= moveArm(approach_pose);

  if (not success) 
  {
    ROS_ERROR("Moving arm to pick approach pose failed");
    return false;
  }

  // open the gripper
  success *= moveGripper(gripper_open_);

  if (not success) 
  {
    ROS_ERROR("Opening gripper prior to pick failed");
    return false;
  }

  // approach to grasping pose
  success *= moveArm(grasp_pose);

  if (not success) 
  {
    ROS_ERROR("Moving arm to grasping pose failed");
    return false;
  }

  // grasp!
  success *= moveGripper(gripper_closed_);

  if (not success) 
  {
    ROS_ERROR("Closing gripper to grasp failed");
    return false;
  }

  // retreat with object
  success *= moveArm(approach_pose);

  if (not success) 
  {
    ROS_ERROR("Retreating arm after picking failed");
    return false;
  }

  success *= moveArm(drop_pose);

  if (not success) 
  {
    ROS_ERROR("Retreating arm after picking failed");
    return false;
  }

  // open the gripper
  success *= moveGripper(gripper_open_);

  if (not success) 
  {
    ROS_ERROR("Opening gripper prior to pick failed");
    return false;
  }

  ROS_INFO("Pick operation successful");

  return true;

}


///////////////////////////////////////////////////////////////////////////////
std::vector<std::string>
cw1::task_2(std::vector<geometry_msgs::PointStamped> basket_locs) 
{
  std::vector<std::string> basket_colours;  

  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);

  geometry_msgs::Pose potential_basket;
  
  bool success = true; 

  success = moveArm(potential_basket);

    if (not success) 
    {
      ROS_ERROR("Retreating arm after picking failed");
      // return false;
    }


  // for (size_t i = 0; i < sizeof(basket_locs); i++)
  for (auto location : basket_locs)
  {
    geometry_msgs::Pose potential_basket;
    potential_basket.orientation = grasp_orientation;
    potential_basket.position = location.point;
    potential_basket.position.z = 0.4;

    bool success = true; 

    success = moveArm(potential_basket);

    if (not success) 
    {
      ROS_ERROR("Retreating arm after picking failed");
      // return false;
    }

    basket_colours.push_back(identify_basket());

  }
  return basket_colours;
}

std::string
cw1::identify_basket()
{
  std::string response = "None";
  return response;
}


// void 
// cw1::imageCallback(const sensor_msgs::ImageConstPtr& msg)
// {
  // auto 
//   // sensor_msgs::CvBridge bridge;
//   // cv_bridge::CvImagePtr cv_ptr;

//   // try
//   // {
//   //   cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
//   //   cv::waitKey(30);
//   // }
//   // catch (cv_bridge::Exception& e)
//   // {
//   //   ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//   // }
//   cv_bridge::CvImagePtr cv_ptr;   // Declare a CvImagePtr 
//   try// Try to convert the ROS image to a CvImage
//   {
//     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//   }
//   catch (cv_bridge::Exception& e)
//   {
//     ROS_ERROR("cv_bridge exception: %s", e.what());
//     return;
//   }
// }

///////////////////////////////////////////////////////////////////////////////
bool 
cw1::task_3()
{

  // Go to viewing pose 

  // define grasping as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);

  geometry_msgs::Pose grasp_pose;
  grasp_pose.orientation = grasp_orientation;
  // experimentally found these values, need to be set
  grasp_pose.position.x = 0.3773;
  grasp_pose.position.y = -0.0015;
  grasp_pose.position.z = 0.8773;

  // set arm position, true if sucessful 
  bool success = moveArm(grasp_pose);

  if (not success) 
  {
    ROS_ERROR("Moving arm to grasping pose failed");
    return false;
  }

  // currently doesn't work, need to fix

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  copyPointCloud(*g_cloud_filtered, *cloud);

  // issue is in this function - possibly due to the fact that the cloud is not being passed by reference or const related
  // check: https://stackoverflow.com/questions/13450838/why-does-pcl-separateclustersextract-require-a-non-const-pointer-to-a-cloud
  separateKMeans(cloud, clusters);

  int counter = 1;
  
  for (auto cluster : clusters)
  {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    // print cluster centroid 
    ROS_INFO("Cluster centroid %x: %f, %f, %f", counter, centroid[0], centroid[1], centroid[2]);
    counter++;
  }

  return success;


}

////////////////////////////////////////////////////////////////////////////////
void
cw1::cloudCallBackOne
  (const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Extract inout point cloud info
  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;
    
  // Convert to PCL data type
  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_cloud_ptr);

  // Perform the filtering
  // applyVX (g_cloud_ptr, g_cloud_filtered);
  applyPT(g_cloud_ptr, g_cloud_filtered);
  
  // Segment plane and cylinder
  //findNormals (g_cloud_filtered);
  //segPlane (g_cloud_filtered);
  // segCylind (g_cloud_filtered);
  //findCylPose (g_cloud_cylinder);
    
  // Publish the data
  //ROS_INFO ("Publishing Filtered Cloud 2");
  pubFilteredPCMsg (g_pub_cloud, *g_cloud_filtered);
  //pubFilteredPCMsg (g_pub_cloud, *g_cloud_cylinder);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
cw1::pubFilteredPCMsg (ros::Publisher &pc_pub,
                               PointC &pc)
{
  // Publish the data
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  pc_pub.publish (g_cloud_filtered_msg);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
cw1::applyPT (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  g_pt.setInputCloud (in_cloud_ptr);
  g_pt.setFilterFieldName ("x");
  g_pt.setFilterLimits(-1.0, 1.0);
  g_pt.setFilterFieldName ("z");
  g_pt.setFilterLimits (g_pt_thrs_min, g_pt_thrs_max);
  g_pt.filter (*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void 
cw1::computeCubeCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
  Eigen::Vector4f &centroid)
{
  pcl::compute3DCentroid(*cloud, centroid);
}

////////////////////////////////////////////////////////////////////////////////
//PointCPtr’         {aka ‘boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> >’} 
// ‘const ConstPtr&’ {aka ‘const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&’}

void
cw1::separateKMeans(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters)
{
  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100); // change these values
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clusters.push_back(cloud_cluster);
  }
}

bool
cw1::passThroughCallback(cw1_team_2::pass_through::Request &request,
  cw1_team_2::pass_through::Response &response)
{
  // set arm position, true if sucessful 
  bool success = true;

  g_pt_thrs_min = request.min;
  g_pt_thrs_max = request.max;

  response.success = success;

  return success;
}

bool 
cw1::setArmCallback(cw1_team_2::set_arm::Request &request,
  cw1_team_2::set_arm::Response &response)
{

  // define grasping as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);

  geometry_msgs::Pose grasp_pose;
  grasp_pose.orientation = grasp_orientation;
  grasp_pose.position.x = 0.3773;
  grasp_pose.position.y = -0.0015;
  grasp_pose.position.z = 0.8773;

  // set arm position, true if sucessful 
  bool success = moveArm(grasp_pose);

  response.success = success;

  return success;
}
