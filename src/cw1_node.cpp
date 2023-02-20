/**
  **********************************************************************************
  * @file     cw1_node.cpp
  * @author   Colin Laganier, Jacob Nash, Carl Parsons
  * @date     2023-02-15
  * @brief   This file contains the main function for the cw1 node. It creates an 
  *          instance of the cw1 class and runs the main loop. 
  **********************************************************************************
  * @attention  Requires cw1_class header file and the ROS backend.
  */

#include <cw1_team_2/cw1_class.h>

int main(int argc, char **argv){
  
  ros::init(argc,argv, "cw1_solution_node");
  ros::NodeHandle nh;

  // create an instance of the cw1 class
  cw1 cw_class(nh);
  
  // MoveIt! requirement for non-blocking group.move()
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // loop rate in Hz
  ros::Rate rate(10);

  while (ros::ok()) {

    // spin and process all pending callbacks
    ros::spinOnce();

    // sleep to fulfill the loop rate
    rate.sleep();
  }
}

