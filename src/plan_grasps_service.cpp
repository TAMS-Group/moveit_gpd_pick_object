#include <ros/ros.h>
#include <moveit_msgs/GraspPlanning.h>
#include <gpd/GraspConfigList.h>
#include <gpd/GraspConfig.h>
#include <mutex>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/move_group_interface/move_group_interface.h>

class PlanGPDGrasp
{
public:
  PlanGPDGrasp(ros::NodeHandle n)
  {
    clustered_grasps_sub_ = n.subscribe("/detect_grasps/clustered_grasps", 1000, &PlanGPDGrasp::clustered_grasps_callback, this);
    pose_pub_ = n.advertise<geometry_msgs::PoseStamped>("tool_pose", 1000);
  }

  void clustered_grasps_callback(const gpd::GraspConfigList::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(m_);
    grasp_candidates_ = *msg;
    geometry_msgs::PoseStamped pose;
    std::string frame_id = grasp_candidates_.header.frame_id;
    for(auto grasp:grasp_candidates_.grasps)
    {
      pose = gpd_grasp_to_pose(grasp, frame_id);
      pose_pub_.publish(pose);
    }

  }

  geometry_msgs::PoseStamped gpd_grasp_to_pose(gpd::GraspConfig grasp, std::string frame_id)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    tf::Matrix3x3 orientation(grasp.approach.x, grasp.approach.y, grasp.approach.z,
                              grasp.binormal.x, grasp.binormal.y, grasp.binormal.z,
                              -grasp.axis.x, -grasp.axis.y, -grasp.axis.z);

    tf::Quaternion orientation_quat;
    orientation.getRotation(orientation_quat);
    geometry_msgs::Quaternion orientation_quat_msg;
    tf::quaternionTFToMsg(orientation_quat, orientation_quat_msg);
    pose.pose.orientation = orientation_quat_msg;

    pose.pose.position = grasp.top;
  }

  bool plan_gpd_grasp(moveit_msgs::GraspPlanning::Request &req, moveit_msgs::GraspPlanning::Response &res)
  {
    moveit::planning_interface::MoveGroupInterface move_group(req.group_name);
    moveit::planning_interface::MoveGroupInterface gripper(move_group.getRobotModel()->getEndEffectors()[0]->getName());

    moveit_msgs::Grasp grasp;
    grasp.id = "grasp";

    jointValuesToJointTrajectory(gripper.getNamedTargetValues("open"), ros::Duration(1.0), grasp.pre_grasp_posture);
    jointValuesToJointTrajectory(gripper.getNamedTargetValues("closed"), ros::Duration(2.0), grasp.grasp_posture);
    std::string frame_id;

    gpd::GraspConfig best_grasp = grasp_candidates_.grasps.at(0);

    {
      std::lock_guard<std::mutex> lock(m_);
      frame_id = grasp_candidates_.header.frame_id;
      grasp.pre_grasp_approach.direction.header.frame_id = frame_id;
      for (auto grasp_candidate:grasp_candidates_.grasps)
      {
        if (grasp_candidate.score.data > best_grasp.score.data)
        {
          best_grasp = grasp_candidate;
        }
      }
    }

    grasp.grasp_pose = gpd_grasp_to_pose(best_grasp, frame_id);

    grasp.pre_grasp_approach.min_distance = 0.08;
    grasp.pre_grasp_approach.desired_distance = 0.1;
    grasp.pre_grasp_approach.direction.vector.x = best_grasp.approach.x;
    grasp.pre_grasp_approach.direction.vector.y = best_grasp.approach.y;
    grasp.pre_grasp_approach.direction.vector.z = best_grasp.approach.z;

    grasp.post_grasp_retreat.min_distance = 0.08;
    grasp.post_grasp_retreat.desired_distance = 0.1;
    grasp.post_grasp_retreat.direction.header.frame_id = move_group.getPlanningFrame();
    grasp.post_grasp_retreat.direction.vector.z = 1.0;

    res.grasps.push_back(grasp);
    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

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

  gpd::GraspConfigList grasp_candidates_;
  std::mutex m_;
  ros::Publisher pose_pub_;
  ros::Subscriber clustered_grasps_sub_;
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
