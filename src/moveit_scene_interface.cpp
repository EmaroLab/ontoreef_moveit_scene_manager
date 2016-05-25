#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <ontoreef_msgs/PerceptionExchange.h>

#include <ros/callback_queue.h>

using namespace ros;
using namespace std;

ros::Publisher* planning_scene_diff_publisher = NULL;

void add_table(){

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "base";

    collision_object.id = "table";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.8;
    primitive.dimensions[1] = 1.6;
    primitive.dimensions[2] = 0.045;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1;
    box_pose.position.x = 0.74;
    box_pose.position.y = 0;
    box_pose.position.z = -0.315;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    ROS_INFO("Adding the object into the world...");
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(collision_object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher->publish(planning_scene);
}


bool update_scene(ontoreef_msgs::PerceptionExchangeRequest& req,
                  ontoreef_msgs::PerceptionExchangeResponse& res){

    ROS_INFO("Received request");

    ROS_INFO("Acquired current state");

    std::vector<pitt_msgs::TrackedShape> objects = req.tracked_shapes;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;

    std::vector<pitt_msgs::TrackedShape>::iterator objectsIterator;

    for(objectsIterator = objects.begin();
        objectsIterator != objects.end(); objectsIterator++){

        pitt_msgs::TrackedShape cur = *objectsIterator;

        if (cur.shape_tag == "sphere"){

            int id = cur.object_id;
            std::string shape = cur.shape_tag;
            shape[0] = toupper(shape[0]);
            float posX = cur.x_est_centroid;
            float posY = cur.y_est_centroid;
            float posZ = cur.z_est_centroid;
            float dim = cur.coefficients[3]; //TODO add support for more primitives

            moveit_msgs::CollisionObject obj;
            obj.header.frame_id = "base";
            obj.id = shape  + "-" + boost::lexical_cast<string>(id);

            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.SPHERE;
            primitive.dimensions.resize(1);
            primitive.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = dim;

            geometry_msgs::Pose pose;
            pose.orientation.w = 1;
            pose.position.x = posX;
            pose.position.y = posY;
            pose.position.z = posZ;

            obj.primitives.push_back(primitive);
            obj.primitive_poses.push_back(pose);
            obj.operation = obj.ADD;

            planning_scene.world.collision_objects.push_back(obj);
        }

        if (cur.shape_tag == "cylinder"){

            int id = cur.object_id;
            std::string shape = cur.shape_tag;
            shape[0] = toupper(shape[0]);
            float posX = cur.x_est_centroid;
            float posY = cur.y_est_centroid;
            float posZ = cur.z_est_centroid;
            float radius = cur.coefficients[6];
            float height = cur.coefficients[7];

            moveit_msgs::CollisionObject obj;
            obj.header.frame_id = "base";
            obj.id = shape  + "-" + boost::lexical_cast<string>(id);

            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = radius;
            primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = height;

            geometry_msgs::Pose pose;
            pose.position.x = posX;
            pose.position.y = posY;
            pose.position.z = posZ;
            pose.orientation.w = 1;

            obj.primitives.push_back(primitive);
            obj.primitive_poses.push_back(pose);
            obj.operation = obj.ADD;

            planning_scene.world.collision_objects.push_back(obj);
        }

//        if (cur.shape_tag == "cone"){
//
//            int id = cur.object_id;
//            std::string shape = cur.shape_tag;
//            shape[0] = toupper(shape[0]);
//            float posX = cur.x_est_centroid;
//            float posY = cur.y_est_centroid;
//            float posZ = cur.z_est_centroid;
//            float radius = cur.coefficients[3]; //TODO add support for more primitives
//            float height = cur.coefficients[4];
//
//            moveit_msgs::CollisionObject obj;
//            obj.header.frame_id = "base";
//            obj.id = shape  + "-" + boost::lexical_cast<string>(id);
//
//            shape_msgs::SolidPrimitive primitive;
//            primitive.type = primitive.CONE;
//            primitive.dimensions.resize(2);
//            primitive.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS] = radius; //TODO convert angle/peak to barycenter/height/radius
//            primitive.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT] = height;
//
//            geometry_msgs::Pose pose;
//            pose.orientation.w = 1;
//            pose.position.x = posX;
//            pose.position.y = posY;
//            pose.position.z = posZ;
//
//            obj.primitives.push_back(primitive);
//            obj.primitive_poses.push_back(pose);
//            obj.operation = obj.ADD;
//
//            planning_scene.world.collision_objects.push_back(obj);
//        }
    }


    planning_scene_diff_publisher->publish(planning_scene);

    //add_table(); //restore table

    ROS_INFO("Acquisition successful");

    res.success = true;
    return true;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "perception_to_moveit_scene");

  ros::NodeHandle node_handle;
  ros::CallbackQueue callback_queue;

  ros::Publisher pub = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  planning_scene_diff_publisher = &pub;

  while(planning_scene_diff_publisher->getNumSubscribers() < 1)
  {
      ros::WallDuration sleep_t(0.5);
      sleep_t.sleep();
  }

  //add_table();

  //ROS_INFO("Table added.");

  ros::AdvertiseServiceOptions opt =
          ros::AdvertiseServiceOptions::create<ontoreef_msgs::PerceptionExchange>
          ("load_moveit_scene", update_scene, ros::VoidPtr(), &callback_queue);
  ros::ServiceServer server = node_handle.advertiseService(opt);

  ros::AsyncSpinner callback_spinner(0, &callback_queue);
  callback_spinner.start();
  ros::spin();

  return 0;
}
