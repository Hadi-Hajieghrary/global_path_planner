#ifndef GLOBAL_PLANNER_H_
#define GLOBAL_PLANNER_H_

#include <vector>
#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>
#include <pluginlib/class_loader.hpp>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "move_base/MoveBaseConfig.h"



namespace global_planner {


    class GlobalPlanner{

        public:
            /**
             * @brief  Constructor
             * 
             * @param tf A reference to a TransformListener
             */
            GlobalPlanner(tf2_ros::Buffer& tf);

            /**
             * @brief  Destructor - Cleans up
             */
            virtual ~GlobalPlanner();

        protected:


        private:

            /**
             * @brief  A service call that that will return a plan
             * 
             * @param  req The goal request
             * @param  resp The plan request
             * 
             * @return True if planning succeeded, false otherwise
             */
            bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

            /**
             * @brief    This will make sure that all the neccessary components is in place 
             *              to call the planner and make a global plan.
             * 
             * @param  start_pose 
             * @param  goal_pose The goal to plan to
             * @param  plan Will be filled in with the plan made by the planner
             * 
             * @return  True if planning succeeds, false otherwise
             */
            bool makePlan(const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& goal_pose, std::vector<geometry_msgs::PoseStamped>& plan);

            /**
             * @brief  Check if a quatenion is constructed well
             * 
             * @param  q The goal to plan to
             */
            bool isQuaternionValid(const geometry_msgs::Quaternion& q);

            /**
             * @brief  Transform the pose to global frame of the costmap
             * 
             * @param  pose 
             * @param  costmap 
             */
            bool transformToGlobalFrame(geometry_msgs::PoseStamped& pose, const costmap_2d::Costmap2DROS* costmap);


            tf2_ros::Buffer& tf_;
            costmap_2d::Costmap2DROS* planner_costmap_ros_;
            boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
            std::string global_planner_ ,global_frame_;
            geometry_msgs::PoseStamped global_pose_;
            ros::Subscriber goal_sub_;
            ros::ServiceServer make_plan_srv_;
            bool shutdown_costmaps_;
            bool make_plan_add_unreachable_goal_;
            pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
            std::vector<geometry_msgs::PoseStamped>* planner_plan_;
            boost::condition_variable_any planner_cond_;
            geometry_msgs::PoseStamped planner_goal_;
    };



}


#endif

