/**
  **********************************************************************************
  * @file     cw1_class.cpp
  * @author   Colin Laganier, Jacob Nash, Carl Parsons
  * @date     2023-02-15
  * @brief   This file contains the constructor and methods for the cw1 class.
  *          The class advertises the services for the coursework tasks and 
  *          triggers the robot to perform the tasks.
  **********************************************************************************
  * @attention  Requires cw1_class header file.
  */

#include <cw1_team_2/cw1_class.h>

////////////////////////////////////////////////////////////////////////////////
// Constructor
////////////////////////////////////////////////////////////////////////////////

cw1::cw1(ros::NodeHandle nh):
  g_cloud_ptr (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
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

  // Define the publishers
  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1, true);
  g_pub_pose = nh.advertise<geometry_msgs::PointStamped> ("cyld_pt", 1, true);
  
  // Initialise ROS Subscribers //
  image_sub_ = nh_.subscribe("/r200/camera/color/image_raw", 1, &cw1::colorImageCallback, this);
  // Create a ROS subscriber for the input point cloud
  cloud_sub_ = nh_.subscribe("/r200/camera/depth_registered/points", 1, &cw1::pointCloudCallback, this);
  

  // Initialising the constants
  load_config();

  ROS_INFO("cw1 class initialised");
}

void cw1::load_config()
{
  // Define constants identified experimentally 
  // Issues with loading from config file (yaml, xml)

  // Pick and place constants
  approach_distance_ = 0.25;
  // Angle offset to align gripper with cube
  angle_offset_ = 3.14159 / 4.0;

  // Pointcloud PassThrough threshold to restrict the pointcloud to the objects
  g_pt_thrs_min = 0.0; 
  g_pt_thrs_max = 0.77; 

  // Defining the Robot scanning position for task 3
  scan_position_.x = 0.3773;
  scan_position_.y = -0.0015;
  scan_position_.z = 0.8773;

  // Positions of the gripper for the different tasks 
  basket_height_ = 0.40;
  cube_height_ = 0.15;
  camera_offset_ = 0.0425;
  // Pointcloud cutoff threshold for object identification
  cube_basket_cutoff_ = 1000;
  // Precision of the estimated object position
  position_precision_ = 1000.0;
}

////////////////////////////////////////////////////////////////////////////////
// Callback functions
////////////////////////////////////////////////////////////////////////////////

bool
cw1::t1_callback(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response) 
{
  /* Function solving Task 1 */

  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  bool success = pickPlace(request.object_loc.pose.position, request.goal_loc.point);
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool 
cw1::t2_callback(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)
{
  /* Function solving Task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  std::vector<std::string> colors = task_2(request.basket_locs);

  response.basket_colours = colors;
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t3_callback(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{
  /* Function solving Task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  bool success = task_3();

  return true;
}

///////////////////////////////////////////////////////////////////////////////

void
cw1::colorImageCallback(const sensor_msgs::Image& msg)
{
  /* This is the callback function for the RGB camera subscriber */ 
  
  // Setting up the static variables at the first callback 
  static bool setup = [&](){
        // Camera feed resolution
        cw1::color_image_width_ = msg.width;
        cw1::color_image_height_ = msg.height;

        // Computing the index of the middle pixel
        cw1::color_image_midpoint_ = cw1::color_channels_ * ((cw1::color_image_width_ * 
          (cw1::color_image_height_ / 2)) + (cw1::color_image_width_ / 2)) - cw1::color_channels_;

        return true;
    } ();

  this->color_image_data = msg.data;

  return;
}

////////////////////////////////////////////////////////////////////////////////

void
cw1::pointCloudCallback
  (const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  /* This is the callback function for the point cloud subscriber */

  // Extract inout point cloud info
  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;
    
  // Convert to PCL data type
  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_cloud_ptr);

  // Perform the filtering
  applyPT(g_cloud_ptr, g_cloud_filtered);
  
  // Publish the data
  pubFilteredPCMsg (g_pub_cloud, *g_cloud_filtered);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
// Point Cloud callback helper functions
////////////////////////////////////////////////////////////////////////////////

void
cw1::pubFilteredPCMsg (ros::Publisher &pc_pub,
                               PointC &pc)
{
  /* This function publishes the filtered pointcloud */

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
  /* Function applying the pass through filter */

  g_pt.setInputCloud (in_cloud_ptr);
  
  // Enlarging the x limits to include the whole plane
  g_pt.setFilterFieldName ("x");
  g_pt.setFilterLimits(-1.0, 1.0);

  // Restricting the z limits to the top of the cubes
  g_pt.setFilterFieldName ("z");
  g_pt.setFilterLimits (g_pt_thrs_min, g_pt_thrs_max);
  
  g_pt.filter (*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
// Task 1
////////////////////////////////////////////////////////////////////////////////

bool
cw1::pickPlace(geometry_msgs::Point object, geometry_msgs::Point target)
{
  /* This function performs a pick and place task given an object position and
  a target position */

  // DEFINE GRIPPER POSES //
  // Grasping Object //
  geometry_msgs::Pose grasp_pose = point2Pose(object);
  grasp_pose.position.z = cube_height_; // Position gripper above object
  // Approach and Takeaway //
  geometry_msgs::Pose offset_pose = grasp_pose;
  offset_pose.position.z += approach_distance_; // Position gripper above object
  // Releasing Object //
  geometry_msgs::Pose release_pose = grasp_pose;
  release_pose.position = target;
  release_pose.position.z = basket_height_; // Position gripper above basket

  // PERFORM PICK //
  bool success = true;
  ROS_INFO("PERFORMING PICK");
  // Aproach object
  success *= moveArm(offset_pose);
  // Open gripper
  success *= moveGripper(gripper_open_);
  // Move to grasp object
  success *= moveArm(grasp_pose);
  // Close gripper
  success *= moveGripper(gripper_closed_);
  // Backaway from object
  success *= moveArm(offset_pose);

  // PERFORM PLACE //
  // Move to basket
  success *= moveArm(release_pose);
  // Open gripper
  success *= moveGripper(gripper_open_);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Task 1 helper functions
////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////

geometry_msgs::Pose
cw1::point2Pose(geometry_msgs::Point point){
  /* This function produces a "gripper facing down" pose given a xyz point */

  // Position gripper above point
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  // Gripper Orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion orientation = tf2::toMsg(q_result);

  // set the desired Pose
  geometry_msgs::Pose pose;
  pose.position = point;
  pose.orientation = orientation;

  return pose;
}

////////////////////////////////////////////////////////////////////////////////
// Task 2
////////////////////////////////////////////////////////////////////////////////

std::vector<std::string>
cw1::task_2(std::vector<geometry_msgs::PointStamped> basket_locs)
{
  /* Function identifying the colour of the baskets based on their location */

  // Determine number of baskets
  int basketNum = basket_locs.size();

  // Initialise string vector (Service Response)
  std::vector<std::string> basket_colors;

  // Initialise output string
  std::string output_string = "Baskets: [";

  // Survey each potential basket location
  for(unsigned int i = 0; i < basketNum; i++){
    // Determine color
    std::string basketColour = survey(basket_locs[i].point);
    // Add decision to vector
    basket_colors.push_back(basketColour);

    // Add to output string
    output_string += basketColour;
    if (i < basketNum - 1){
      output_string += ", ";
    }
  }

  // Print results
  output_string += "]";
  ROS_INFO("/////////////////////////////////////////////////////////////////////");
  ROS_INFO("%s", output_string.c_str());
  ROS_INFO("/////////////////////////////////////////////////////////////////////");

  return basket_colors;
}

////////////////////////////////////////////////////////////////////////////////
// Task 2 helper functions
////////////////////////////////////////////////////////////////////////////////

std::string
cw1::survey(geometry_msgs::Point point)
{
  /* This function will return the color shown in camera, given a point of interest*/

  // Define imaging pose
  geometry_msgs::Pose image_pose = point2Pose(point);
  image_pose.position.z += 0.3; //Offset above object
  image_pose.position.x -= 0.04; //Offset of camera from end-effector
  
  // Move camera above object
  bool success = moveArm(image_pose);
  // Extract central pixel values from raw RGB data (Obtained from subsribed topic)
  int redValue = color_image_data[color_image_midpoint_];
  int greenValue = color_image_data[color_image_midpoint_ + 1];
  int blueValue = color_image_data[color_image_midpoint_ + 2];

  // Determine Colour of Basket
  std::string basketColour;
  if (greenValue > redValue && greenValue > blueValue){
    basketColour = "none";
    ROS_INFO("NONE");
  } else if (redValue > greenValue && redValue > blueValue){
    basketColour = "red";
    ROS_INFO("RED");
  } else if (blueValue > redValue && blueValue > greenValue){
    basketColour = "blue";
    ROS_INFO("BLUE");
  } else if (redValue == blueValue){
    basketColour = "purple";
    ROS_INFO("PURPLE");
  } else{
    basketColour = "Failed To Determine";
  }
  
  return basketColour;
}


////////////////////////////////////////////////////////////////////////////////
// Task 3
////////////////////////////////////////////////////////////////////////////////

bool 
cw1::task_3()
{
  /* Function identifying cubes in the environment and sorting them in the 
     correct basket */

  // Move robot and camera to scannning pose 
  geometry_msgs::Pose scan_pose = point2Pose(scan_position_);
  bool success = true;
  success *= moveArm(scan_pose);

  ROS_INFO("Starting to cluster");

  // Create snapshot of point cloud for processing
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  copyPointCloud(*g_cloud_filtered, *cloud);

  // Cluster the point cloud into separate objects
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clusters = clusterPointclouds(cloud);

  ROS_INFO("Finished clustering");

  // Create a vector of tuples containing the centroid position and color of each cluster
  std::vector<std::tuple<geometry_msgs::Point, Color>> cube_data;
  std::vector<std::tuple<geometry_msgs::Point, Color>> basket_data;

  ROS_INFO("Starting object identification");
  
  for (auto cluster : clusters)
  {
    // Compute the centroid of the cluster
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);

    // Create a point to store the centroid position
    geometry_msgs::Point centroid_position;

    // Round to 3 decimal places
    centroid_position.x = std::round(centroid[0] * position_precision_) / position_precision_;
    centroid_position.y = std::round(centroid[1] * position_precision_) / position_precision_;
    centroid_position.z = std::round(centroid[2] * position_precision_) / position_precision_;

    // Pick a random point in the cluster to determine the color of the cluster
    int random_point = rand() % cluster->size();;
    uint32_t rgb = *reinterpret_cast<int*>(&cluster->points[random_point].rgb);

    // Identifies the color of the cluster
    Color cluster_color = identifyColor(rgb);

    // Identifies if cluster is a cube or a box and adds it to the correct vector
    if (cluster->size() < cube_basket_cutoff_)
    {
      cube_data.push_back(std::make_tuple(centroid_position, cluster_color));
    }
    else
    {
      basket_data.push_back(std::make_tuple(centroid_position, cluster_color));
    }
  }

  ROS_INFO("Finished object identification");
  ROS_INFO("Starting pick and place operation");

  // Loop through all cubes
  for (auto cube : cube_data)
  {
    // Compute cube pose
    geometry_msgs::Point cube_point;
    cube_point.x = scan_position_.x - std::get<0>(cube).y + camera_offset_;
    cube_point.y = scan_position_.y - std::get<0>(cube).x;
    cube_point.z = 0.0;

    // Identify target basket based on current cube color and position
    TargetBasket target_basket = identifyBasket(cube, basket_data);

    // Verify that a basket was found for given cube
    if (target_basket.is_empty)
    {
      ROS_INFO("No basket matching cube color found");
      continue;
    }

    // Pick and place cube in target basket
    success *= pickPlace(cube_point, target_basket.coordinates);
  }

  ROS_INFO("Finished pick and place operation");

  return success;
}

////////////////////////////////////////////////////////////////////////////////
// Task 3 helper functions
////////////////////////////////////////////////////////////////////////////////

std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
cw1::clusterPointclouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
  /* Function clustering point clouds in individual groups */

  // Vector to store the clusters
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clusters;

  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
  tree->setInputCloud(cloud);

  // Create a set of indices to be used in the extraction
  std::vector<pcl::PointIndices> cluster_indices;
  // Create the extraction object for the clusters
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> cluster_extraction;
  // Set the extraction parameters
  cluster_extraction.setClusterTolerance(0.02); // 2cm
  cluster_extraction.setMinClusterSize(100);  
  cluster_extraction.setMaxClusterSize(10000);
  cluster_extraction.setSearchMethod(tree);
  cluster_extraction.setInputCloud(cloud);
  cluster_extraction.extract(cluster_indices);

  // Loop through each cluster and store it in the vector
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (const auto& idx : cluster.indices) {
      cloud_cluster->push_back((*cloud)[idx]);
    }
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    ROS_INFO("PointCloud representing the Cluster: %lu data points.", cloud_cluster->size());
    clusters.push_back(cloud_cluster);
  }

  return clusters;
}

///////////////////////////////////////////////////////////////////////////////

cw1::Color cw1::identifyColor(uint32_t rgb)
{
  /* Function classifying the cube colour based on its rgb value */

  // Unpack RGB values from point
  uint8_t r = (rgb >> 16) & 0x0000ff;
  uint8_t g = (rgb >> 8)  & 0x0000ff;
  uint8_t b = (rgb)       & 0x0000ff;

  // Identifies the color of the cluster based on the RGB values
  if (g > r && g > b)
  {
    return Color::none;
  }
  else if (r > g && r > b)
  {
    return Color::red;
  }
  else if (b > r && b > g)
  {
    return Color::blue;
  }
  else if (r == b)
  {
    return Color::purple;
  } 
  else
  {
    return Color::none;
  }
};

///////////////////////////////////////////////////////////////////////////////

cw1::TargetBasket 
cw1::identifyBasket(std::tuple<geometry_msgs::Point, Color> cube, 
  std::vector<std::tuple<geometry_msgs::Point, Color>> &basket_data)
{
  /* Function matching a given cube with its corresponding basket */

  // Create a target basket object to store the basket position and distance to cube
  TargetBasket target_basket; 
  // Loop through every baskets
  for (const auto &basket : basket_data)
  {
    // If basket and cube are the same color
    if (std::get<1>(basket) == std::get<1>(cube))
    {
      // Calculate distance between cube and basket
      float distance_to_cube = sqrt(pow(std::get<0>(basket).x - std::get<0>(cube).x, 2) 
        + pow(std::get<0>(basket).y - std::get<0>(cube).y, 2));

      // If ditance to basket is smaller than current target basket, update target basket
      if (distance_to_cube < target_basket.distance_to_cube)
      {
        // Computes position of basket relative to camera
        target_basket.coordinates.x = scan_position_.x - std::get<0>(basket).y + camera_offset_;
        target_basket.coordinates.y = scan_position_.y - std::get<0>(basket).x;
        target_basket.coordinates.z = 0.0;
        target_basket.distance_to_cube = distance_to_cube;
        target_basket.is_empty = false;
      }
    }
  }

  return target_basket;
}
