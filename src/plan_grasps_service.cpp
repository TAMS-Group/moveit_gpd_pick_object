#include <ros/ros.h>
#include <moveit_msgs/GraspPlanning.h>
#include <moveit_msgs/CollisionObject.h>
#include <gpd/GraspConfigList.h>
#include <gpd/GraspConfig.h>
#include <mutex>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <moveit/move_group_interface/move_group_interface.h>

class PlanGPDGrasp
{
public:
  PlanGPDGrasp(ros::NodeHandle &n)
  { 
    clustered_grasps_sub_ = n.subscribe("/detect_grasps/clustered_grasps", 1000, &PlanGPDGrasp::clustered_grasps_callback, this);
    grasps_visualization_pub_ = n.advertise<geometry_msgs::PoseArray>("grasps_visualization", 10);
    ros::NodeHandle pn("~");

    pn.param("bound_frame", bound_frame_, std::string("table_top"));
    pn.param("x_bound", x_bound_, 0.60);
    pn.param("y_bound", y_bound_, 0.80);
    pn.param("z_bound", z_bound_, 0.50);
    pn.param("x_bound_offset", x_bound_offset_, 0.0);
    pn.param("y_bound_offset", y_bound_offset_, 0.0);
    pn.param("z_bound_offset", z_bound_offset_, 0.29);

    pn.param("grasp_offset", grasp_offset_, -0.08);

    pn.param("grasp_cache_time_threshold", grasp_cache_time_threshold_, 5.0);

    std::string move_group_arm;
    std::string move_group_gripper;
    pn.param("move_group_arm", move_group_arm, std::string("arm"));
    pn.param("move_group_gripper", move_group_gripper, std::string("gripper"));
 
    moveit::planning_interface::MoveGroupInterface move_group(move_group_arm);
    moveit::planning_interface::MoveGroupInterface gripper(move_group_gripper);
 
    // Setting variables in grasp_candidate_ that are the same for every grasp
    grasp_candidate_.id = "grasp";

    grasp_candidate_.pre_grasp_approach.min_distance = 0.08;
    grasp_candidate_.pre_grasp_approach.desired_distance = 0.1;

    grasp_candidate_.post_grasp_retreat.min_distance = 0.13;
    grasp_candidate_.post_grasp_retreat.desired_distance = 0.15;
    grasp_candidate_.post_grasp_retreat.direction.header.frame_id = move_group.getPlanningFrame();
    grasp_candidate_.post_grasp_retreat.direction.vector.z = 1.0;

    jointValuesToJointTrajectory(gripper.getNamedTargetValues("open"), ros::Duration(1.0), grasp_candidate_.pre_grasp_posture);
    jointValuesToJointTrajectory(gripper.getNamedTargetValues("closed"), ros::Duration(2.0), grasp_candidate_.grasp_posture);
  }

  void clustered_grasps_callback(const gpd::GraspConfigList::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(m_);
    ros::Time grasp_stamp = msg->header.stamp;
    frame_id_ = msg->header.frame_id;
    grasp_candidate_.grasp_pose.header.frame_id = frame_id_;
    grasp_candidate_.pre_grasp_approach.direction.header.frame_id = frame_id_;

    for(auto grasp:msg->grasps)
    {
      // shift the grasp according to the offset parameter
      grasp.top.x = grasp.top.x + grasp_offset_ * grasp.approach.x;
      grasp.top.y = grasp.top.y + grasp_offset_ * grasp.approach.y;
      grasp.top.z = grasp.top.z + grasp_offset_ * grasp.approach.z;

      if(grasp_boundry_check(grasp))
      {
        grasp_candidate_.grasp_pose.pose = gpd_grasp_to_pose(grasp);

        grasp_candidate_.grasp_quality = grasp.score.data;
        grasp_candidate_.pre_grasp_approach.direction.vector.x = grasp.approach.x;
        grasp_candidate_.pre_grasp_approach.direction.vector.y = grasp.approach.y;
        grasp_candidate_.pre_grasp_approach.direction.vector.z = grasp.approach.z;

        grasp_candidates_.push_front(std::make_pair(grasp_candidate_, grasp_stamp));
      }
    }
  }

  bool grasp_boundry_check(gpd::GraspConfig &grasp)
  {
    geometry_msgs::PointStamped grasp_point;
    geometry_msgs::PointStamped transformed_grasp_point;
    grasp_point.header.frame_id = frame_id_;
 
    grasp_point.point = grasp.top;
    // transform the grasp point into the frame of the bound to make the boundary check easier
    try
    {
      listener_.transformPoint(bound_frame_, grasp_point, transformed_grasp_point);
    }
    catch( tf::TransformException ex)
    {
      ROS_ERROR("transfrom exception : %s",ex.what());
    }

    return (transformed_grasp_point.point.x < x_bound_*0.5+x_bound_offset_
         && transformed_grasp_point.point.x > -x_bound_*0.5+x_bound_offset_
         && transformed_grasp_point.point.y < y_bound_*0.5+y_bound_offset_
         && transformed_grasp_point.point.y > -y_bound_*0.5+y_bound_offset_
         && transformed_grasp_point.point.z < z_bound_*0.5+z_bound_offset_
         && transformed_grasp_point.point.z > -z_bound_*0.5+z_bound_offset_);
  }

  geometry_msgs::Pose gpd_grasp_to_pose(gpd::GraspConfig &grasp)
  {
    geometry_msgs::Pose pose;
    tf::Matrix3x3 orientation(grasp.approach.x, grasp.binormal.x, grasp.axis.x,
                              grasp.approach.y, grasp.binormal.y, grasp.axis.y,
                              grasp.approach.z, grasp.binormal.z, grasp.axis.z);

    tf::Quaternion orientation_quat;
    orientation.getRotation(orientation_quat);
    tf::quaternionTFToMsg(orientation_quat, pose.orientation);

    pose.position = grasp.top;

    return pose;
  }

  bool plan_gpd_grasp(moveit_msgs::GraspPlanning::Request &req, moveit_msgs::GraspPlanning::Response &res)
  {
    geometry_msgs::PoseArray grasps_visualization;
    grasps_visualization.header.frame_id = frame_id_;
 
    {
      std::lock_guard<std::mutex> lock(m_);
      for (auto grasp_candidate:grasp_candidates_)
      {
        // after the first grasp older than the set amount of seconds is found, break the loop
        if(grasp_candidate.second.sec < ros::Time::now().sec - grasp_cache_time_threshold_)
          break;
        res.grasps.push_back(grasp_candidate.first);
        grasps_visualization.poses.push_back(grasp_candidate.first.grasp_pose.pose);
      }
      grasp_candidates_.clear();
    }

    if(res.grasps.empty())
    {
      ROS_INFO("No valid grasp found.");
      res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    else
    {
      ROS_INFO("%ld grasps found.", res.grasps.size());
      res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
    grasps_visualization_pub_.publish(grasps_visualization);

    return true;
  }

private:
  void jointValuesToJointTrajectory(std::map<std::string, double> target_values, ros::Duration duration,
        trajectory_msgs::JointTrajectory &grasp_pose)
  {
    grasp_pose.joint_names.reserve(target_values.size());
    grasp_pose.points.resize(1);
    grasp_pose.points[0].positions.reserve(target_values.size());

    for(std::map<std::string, double>::iterator it = target_values.begin(); it != target_values.end(); ++it)
    {
      grasp_pose.joint_names.push_back(it->first);
      grasp_pose.points[0].positions.push_back(it->second);
    }
    grasp_pose.points[0].time_from_start = duration;
  }

  moveit_msgs::Grasp grasp_candidate_;
  std::deque<std::pair<moveit_msgs::Grasp, ros::Time>> grasp_candidates_;
  std::mutex m_;
  ros::Subscriber clustered_grasps_sub_;
  tf::TransformListener listener_;
  ros::Publisher grasps_visualization_pub_;

  // frame of the grasp
  std::string frame_id_;

  // frame of the bound that determines which grasps are valid
  std::string bound_frame_;
 
  // measurements of the bound that determines which grasps are valid
  double x_bound_;
  double y_bound_;
  double z_bound_;
 
  // offset of the bounds from the 0 position
  double x_bound_offset_;
  double y_bound_offset_;
  double z_bound_offset_;

  // offset of the grasp along the approach vector
  double grasp_offset_;

  // grasps older than this threshold are not considered anymore
  double grasp_cache_time_threshold_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_grasps_service");
  ros::NodeHandle n;
  PlanGPDGrasp p_gpd_g(n);

  ros::ServiceServer ss = n.advertiseService("plan_grasps", &PlanGPDGrasp::plan_gpd_grasp, &p_gpd_g);
  ros::spin();

  return 0;
}

