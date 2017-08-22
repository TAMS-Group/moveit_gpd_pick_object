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
    ros::NodeHandle pn("~");
    pn.param("frame_id", frame_id_, std::string("kinect2_rgb_optical_frame"));
    grasps_visualization_pub_ = pn.advertise<geometry_msgs::PoseArray>( "grasps_visualization", 10 );
  }

  void clustered_grasps_callback(const gpd::GraspConfigList::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(m_);
    grasp_candidates_.grasps.insert(grasp_candidates_.grasps.begin(), msg->grasps.begin(), msg->grasps.end());
    
    // Cache up to 20 grasps. In case there are more grasps the oldest cached grasps get deleted.
    if(grasp_candidates_.grasps.size() >= 21)
      grasp_candidates_.grasps.erase(grasp_candidates_.grasps.begin()+21,  grasp_candidates_.grasps.end());
  }

  geometry_msgs::Pose gpd_grasp_to_pose(gpd::GraspConfig grasp)
  {
    geometry_msgs::Pose pose;
    tf::Matrix3x3 orientation(grasp.approach.x, grasp.binormal.x, grasp.axis.x,
                              grasp.approach.y, grasp.binormal.y, grasp.axis.y,
                              grasp.approach.z, grasp.binormal.z, grasp.axis.z);

    tf::Quaternion orientation_quat;
    orientation.getRotation(orientation_quat);
    geometry_msgs::Quaternion orientation_quat_msg;
    tf::quaternionTFToMsg(orientation_quat, orientation_quat_msg);
    pose.orientation = orientation_quat_msg;
    double factor = -0.06;
    pose.position.x = grasp.top.x + factor * grasp.approach.x;
    pose.position.y = grasp.top.y + factor * grasp.approach.y;
    pose.position.z = grasp.top.z + factor * grasp.approach.z;

    return pose;
  }

  bool plan_gpd_grasp(moveit_msgs::GraspPlanning::Request &req, moveit_msgs::GraspPlanning::Response &res)
  {
    moveit::planning_interface::MoveGroupInterface move_group(req.group_name);
    moveit::planning_interface::MoveGroupInterface gripper(move_group.getRobotModel()->getEndEffectors()[0]->getName());
    moveit_msgs::CollisionObject co = req.target;
    double x_bound = co.primitives[0].dimensions[0]/2.0;
    double y_bound = co.primitives[0].dimensions[1]/2.0;
    double z_bound = co.primitives[0].dimensions[2];
    
    moveit_msgs::Grasp grasp;
    grasp.id = "grasp";
    grasp.grasp_pose.header.frame_id = frame_id_;
    
    grasp.pre_grasp_approach.min_distance = 0.08;
    grasp.pre_grasp_approach.desired_distance = 0.1;
    grasp.pre_grasp_approach.direction.header.frame_id = frame_id_;
    
    grasp.post_grasp_retreat.min_distance = 0.08;
    grasp.post_grasp_retreat.desired_distance = 0.1;
    grasp.post_grasp_retreat.direction.header.frame_id = move_group.getPlanningFrame();
    grasp.post_grasp_retreat.direction.vector.z = 1.0;

    jointValuesToJointTrajectory(gripper.getNamedTargetValues("open"), ros::Duration(1.0), grasp.pre_grasp_posture);
    jointValuesToJointTrajectory(gripper.getNamedTargetValues("closed"), ros::Duration(2.0), grasp.grasp_posture);

    geometry_msgs::PointStamped grasp_point;
    grasp_point.header.frame_id = frame_id_;
    geometry_msgs::PointStamped transformed_grasp_point;
    geometry_msgs::PoseArray grasps_visualization;
    grasps_visualization.header.frame_id = frame_id_;
    
    {
      std::lock_guard<std::mutex> lock(m_);
      for (auto grasp_candidate:grasp_candidates_.grasps)
      {
        grasp_point.point = grasp_candidate.top;
        try
        {
          listener_.transformPoint(co.header.frame_id, grasp_point, transformed_grasp_point); 
        }
        catch( tf::TransformException ex)
        {
          ROS_ERROR("transfrom exception : %s",ex.what());
        }

        if (transformed_grasp_point.point.x < x_bound && transformed_grasp_point.point.x > -x_bound && transformed_grasp_point.point.y < y_bound && transformed_grasp_point.point.y > -y_bound && transformed_grasp_point.point.z < 0.05+z_bound && transformed_grasp_point.point.z > 0.05)
        { 
          grasp.grasp_pose.pose = gpd_grasp_to_pose(grasp_candidate);
          grasps_visualization.poses.push_back(grasp.grasp_pose.pose);

          grasp.grasp_quality = grasp_candidate.score.data;

          grasp.pre_grasp_approach.direction.vector.x = grasp_candidate.approach.x;
          grasp.pre_grasp_approach.direction.vector.y = grasp_candidate.approach.y;
          grasp.pre_grasp_approach.direction.vector.z = grasp_candidate.approach.z;

          res.grasps.push_back(grasp);
        }
      }
    }
    grasp_candidates_.grasps.clear();
    if(res.grasps.empty())
    {
      ROS_ERROR("No valid grasp found.");
      res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return false;
    }
    ROS_INFO("Grasps found.");
    grasps_visualization_pub_.publish(grasps_visualization);
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
  ros::Subscriber clustered_grasps_sub_;
  tf::TransformListener listener_;
  std::string frame_id_;
  ros::Publisher grasps_visualization_pub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_grasps_service");
  ros::NodeHandle n;
  PlanGPDGrasp p_gpd_g(n);

  ros::ServiceServer ss = n.advertiseService("plan_grasps", &PlanGPDGrasp::plan_gpd_grasp, &p_gpd_g);
  while(ros::ok())
  {
    ros::spin();
  }

  return 0;
}

