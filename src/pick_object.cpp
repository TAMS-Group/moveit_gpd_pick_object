#include <iostream>
#include <stdio.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/collision_detection/collision_matrix.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PickupActionResult.h>

#include <manipulation_msgs/GraspPlanning.h>

class PickObject
{
public:
  moveit::planning_interface::MoveGroupInterface arm;
  moveit::planning_interface::MoveGroupInterface gripper;
  moveit::planning_interface::PlanningSceneInterface psi;
  bool placed_object;
  bool success;

  PickObject() : arm("arm"), gripper("gripper"), placed_object(true), success(false)
  {
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;
    
    pnh.param("box_x", box_x_, 0.60);
    pnh.param("box_y", box_y_, 0.80);
    pnh.param("box_z", box_z_, 0.50);
    pnh.param("box_z_offset", box_z_offset_, 0.05);
  }

  ~PickObject()
  {
  }

  bool executePick()
  {
    arm.setPlanningTime(20.0);
    arm.setSupportSurfaceName("table");
    success = arm.planGraspsAndPick("bounding_box");
    return success;
  }

  bool executePick(moveit_msgs::CollisionObject &object)
  {
    arm.setPlanningTime(20.0);
    arm.setSupportSurfaceName("table");
    success = arm.planGraspsAndPick(object);
    return success;
  }

  /**
   * Spawns a bounding box on the table. Only grasps inside the box are seen as valid.
   */
  moveit_msgs::CollisionObject spawnBoundingBox()
  {
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.robot_state.is_diff = true;

    moveit_msgs::CollisionObject bounding_box;

    bounding_box.header.frame_id = "table_top";
    bounding_box.id = "bounding_box";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.push_back(box_x_);
    primitive.dimensions.push_back(box_y_);
    primitive.dimensions.push_back(box_z_);

    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = primitive.dimensions[2] / 2 + box_z_offset_;

    bounding_box.primitives.push_back(primitive);
    bounding_box.primitive_poses.push_back(pose);

    // add object to scene
    bounding_box.operation = bounding_box.ADD;
    planning_scene.world.collision_objects.push_back(bounding_box);

     // remove attached object in case it is attached
    moveit_msgs::AttachedCollisionObject aco;
    bounding_box.operation = bounding_box.REMOVE;
    aco.object = bounding_box;
    planning_scene.robot_state.attached_collision_objects.push_back(aco);
    
    // apply the new object
    psi.applyPlanningScene(planning_scene);

    moveit_msgs::PlanningScene current_scene;
    moveit_msgs::PlanningScene new_planning_scene;
    new_planning_scene.is_diff = true;
    new_planning_scene.robot_state.is_diff = true;

    moveit_msgs::GetPlanningScene scene_srv;
    
    ros::NodeHandle nh;
    ros::ServiceClient client_get_scene = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    scene_srv.request.components.components = scene_srv.request.components.ALLOWED_COLLISION_MATRIX;
    
    if(!client_get_scene.call(scene_srv))
    {
      ROS_WARN("Failed to call service /get_planning_scene");
    }
    else
    {
      current_scene = scene_srv.response.scene;
      collision_detection::AllowedCollisionMatrix acm(current_scene.allowed_collision_matrix);
      std::vector<std::string> entries;
      acm.getAllEntryNames(entries);
      acm.setEntry("bounding_box", entries, true);
      acm.setDefaultEntry("bounding_box", true);
      acm.getMessage(new_planning_scene.allowed_collision_matrix);

      psi.applyPlanningScene(new_planning_scene);
    }

    return bounding_box;
  }
private:
  void jointValuesToJointTrajectory(std::map<std::string, double> target_values, trajectory_msgs::JointTrajectory &grasp_pose)
  {
    grasp_pose.joint_names.reserve(target_values.size());
    grasp_pose.points.resize(1);
    grasp_pose.points[0].positions.reserve(target_values.size());

    for(std::map<std::string, double>::iterator it = target_values.begin(); it != target_values.end(); ++it)
    {
      grasp_pose.joint_names.push_back(it->first);
      grasp_pose.points[0].positions.push_back(it->second);
    }
  }
  double box_x_;
  double box_y_;
  double box_z_;
  double box_z_offset_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_object");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  PickObject po;
  ROS_INFO("Spawn Bounding Box");
  moveit_msgs::CollisionObject object = po.spawnBoundingBox();
  moveit_msgs::AttachedCollisionObject aco;
  aco.object = object;
  aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
  
  ROS_INFO("Picking Object");
  while(ros::ok())
  {
    if(po.success)
    {
      po.arm.setNamedTarget("extended");
      po.arm.move();
      po.gripper.setNamedTarget("open");
      po.gripper.move();
      po.psi.applyAttachedCollisionObject(aco);
      po.spawnBoundingBox();
      po.arm.setNamedTarget("home");
      po.arm.move();
      ros::Duration(10).sleep();
    }
    po.executePick(object);
  }

  return 0;
}

