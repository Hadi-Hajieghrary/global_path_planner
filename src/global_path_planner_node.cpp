#include <global_path_planner/global_path_planner.h>
#include <tf2_ros/transform_listener.h>



int main(int argc, char** argv){
  ros::init(argc, argv, "global_path_planner_node");
  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  global_planner::GlobalPlanner global_planner( buffer );
  

  ros::spin();

  return(0);
  
}
