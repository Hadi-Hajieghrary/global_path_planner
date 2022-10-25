

#include <global_path_planner/global_path_planner.h>

using namespace global_planner;

// Public:

GlobalPlanner::GlobalPlanner(tf2_ros::Buffer& tf) :
  tf_(tf),
  planner_costmap_ros_(NULL),
  bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
  planner_plan_(NULL){
  
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  private_nh.param("base_global_planner", global_planner_, std::string("navfn/NavfnROS"));
  private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);
  private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));

  //set up plan triple buffer
  planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();

  //create the ros wrapper for the planner's costmap
  planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
  planner_costmap_ros_->pause();

  //initialize the global planner
  try {
    planner_ = bgp_loader_.createInstance(global_planner_);
    planner_->initialize(bgp_loader_.getName(global_planner_), planner_costmap_ros_);
  } catch (const pluginlib::PluginlibException& ex) {
    ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner_.c_str(), ex.what());
    exit(1);
  }

  // Start actively updating costmaps based on sensor data
  planner_costmap_ros_->start();

  //advertise a service for getting a plan
  make_plan_srv_ = private_nh.advertiseService("make_plan", &GlobalPlanner::planService, this);
}


GlobalPlanner::~GlobalPlanner(){
    if(planner_costmap_ros_ != nullptr)  delete planner_costmap_ros_;
    planner_costmap_ros_ = nullptr;

    if(planner_plan_ != nullptr)  delete planner_plan_;
    planner_plan_ = nullptr;

    planner_.reset();
}


// Private:

bool GlobalPlanner::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){
  ros::spinOnce();
  //make sure we have a costmap for our planner
  if(planner_costmap_ros_ == NULL){
    ROS_ERROR("Planner cannot make a plan for you because it doesn't have a costmap");
    return false;
  }

  geometry_msgs::PoseStamped start{req.start};
  geometry_msgs::PoseStamped goal{req.goal};

  //first try to make a plan to the exact desired goal
  std::vector<geometry_msgs::PoseStamped> global_plan;
  if(!planner_->makePlan(start, goal, global_plan) || global_plan.empty()){
    ROS_DEBUG_NAMED("global_planner","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
                      goal.pose.position.x, goal.pose.position.y);

    //search outwards for a feasible goal within the specified tolerance
    geometry_msgs::PoseStamped p{req.goal};
    bool found_legal = false;
    float resolution = planner_costmap_ros_->getCostmap()->getResolution();
    float search_increment = resolution*3.0;

    if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;

    for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
      for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
        for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

          //don't search again inside the current outer layer
          if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

          //search to both sides of the desired goal
          for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

            //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
            if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

            for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
              if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

              p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
              p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

              if(planner_->makePlan(start, p, global_plan)){
                if(!global_plan.empty()){

                  if (make_plan_add_unreachable_goal_) {
                    //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                    //(the reachable goal should have been added by the global planner)
                    global_plan.push_back(req.goal);
                  }

                  found_legal = true;
                  ROS_DEBUG_NAMED("global_planner", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                  break;
                }
              }
              else{
                ROS_DEBUG_NAMED("global_planner","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
              }
            }
          }
        }
      }
    }
  }

  //copy the plan into a message to send out
  resp.plan.poses.resize(global_plan.size());
  for(unsigned int i = 0; i < global_plan.size(); ++i){
    resp.plan.poses[i] = global_plan[i];
  }

  return true;
}


bool GlobalPlanner::isQuaternionValid(const geometry_msgs::Quaternion& q){
  //first we need to check if the quaternion has nan's or infs
  if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
    ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
    return false;
  }

  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

  //next, we need to check if the length of the quaternion is close to zero
  if(tf_q.length2() < 1e-6){
    ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
    return false;
  }

  //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
  tf_q.normalize();

  tf2::Vector3 up(0, 0, 1);

  double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

  if(fabs(dot - 1) > 1e-3){
    ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
    return false;
  }

  return true;
}


bool GlobalPlanner::transformToGlobalFrame(geometry_msgs::PoseStamped& pose, const costmap_2d::Costmap2DROS* costmap){

  geometry_msgs::PoseStamped cur_pose = pose;
  std::string frame = costmap->getGlobalFrameID();
  try{
    tf_.transform(cur_pose, pose, global_frame_);
    pose.header.stamp = ros::Time();
    pose.header.frame_id = frame;
  }catch(std::exception& ex){
    ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
        pose.header.frame_id.c_str(), frame.c_str(), ex.what());
    return false;
  }
    return true;
}


bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& goal_pose, std::vector<geometry_msgs::PoseStamped>& plan){
  //lock access to the costmap
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

  if(lock.try_lock()){

    //make sure to set the plan to be empty initially
    plan.clear();

    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL) {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      lock.unlock();
      return false;
    }

    //get the starting pose of the robot
    geometry_msgs::PoseStamped global_start_pose{start_pose};
    geometry_msgs::PoseStamped global_goal_pose{goal_pose};
    if(!transformToGlobalFrame(global_start_pose, planner_costmap_ros_) || !transformToGlobalFrame(global_goal_pose, planner_costmap_ros_)) {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      lock.unlock();
      return false;
    }

    //if the planner fails or returns a zero length plan, planning failed
    if(!planner_->makePlan(start_pose, goal_pose, plan) || plan.empty()){
      ROS_DEBUG_NAMED("global_planner","Failed to find a  plan to point (%.2f, %.2f)", goal_pose.pose.position.x, goal_pose.pose.position.y);
      lock.unlock();
      return false;
    }
  }
  lock.unlock();
  return true;
}
