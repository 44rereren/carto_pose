#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32MultiArray.h>

#define LIDAR_DIST_X 0.39171866199f //Lidarとロボット中心のX軸距離[m]
#define LIDAR_DIST_Y 0.f //Lidarとロボット中心のY軸距離[m]
#define INIT_X 4563.f//4547.395f//ロボットの初期座標[mm]
#define INIT_Y -9516.f //9381.325f //ロボットの初期座標[mm]
#define FREQUENCY 100 //[Hz]
#define A 0.5f //加重平均フィルタの係数

typedef struct
{
  float x;
  float y;
  float theta;
}odom_t;

typedef struct
{
  float x;
  float y;
  float theta;
}carto_t;

typedef struct
{
  float x;
  float y;
  float theta;
}robot_t;

odom_t odom;
carto_t carto;
robot_t robot;

float lidar_dist() //Lidarとロボット中心の距離[m]
{
  return sqrt(powf(LIDAR_DIST_X, 2) + powf(LIDAR_DIST_Y, 2));
}

float weightAverageFilter(float val_1, float val_2)
{
  return A * val_1 + (1.f - A) * val_2;
}

void callback(const std_msgs::Float32MultiArray &msg)
{
  odom.x = msg.data[0]; //[mm]
  odom.y = msg.data[1]; //[mm]
  odom.theta = msg.data[2]; //[rad]  default: cw -> +, ccw -> -
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_pose");
  ros::NodeHandle nh;
  ros::Rate rate(FREQUENCY);
  tf::TransformListener listener;
  ros::Subscriber stm_sub = nh.subscribe("stm_data", 1000, callback);
  ros::Publisher robot_pub = nh.advertise<std_msgs::Float32MultiArray>("robot_pose",100);

  std_msgs::Float32MultiArray robot_array;
  robot_array.data.resize(2);

  while(ros::ok())
  {
    tf::StampedTransform transform;
    ros::spinOnce();
    try
    {
      listener.waitForTransform("/map", "/laser", ros::Time(0), ros::Duration(1)); 
      listener.lookupTransform("/map", "/laser", ros::Time(0), transform);
      carto.theta = odom.theta; //[rad]
      carto.x = INIT_X - (transform.getOrigin().x() - (lidar_dist() * cosf(carto.theta))) * 1000.f; //[mm]
      carto.y = INIT_Y + (transform.getOrigin().y() + (lidar_dist() * sinf(carto.theta))) * 1000.f; //[mm]
      //carto.x = INIT_X - transform.getOrigin().x() * 1000.f; //[mm]
      //carto.y = INIT_Y + transform.getOrigin().y() * 1000.f; //[mm]
      robot.theta = odom.theta; //[rad]
      robot.x = weightAverageFilter(odom.x, carto.x); //[mm]
      robot.y = weightAverageFilter(odom.y, carto.y); //[mm]

      robot_array.data[0] = robot.x;
      robot_array.data[1] = robot.y;
      robot_pub.publish(robot_array);

      printf("c:%.0f %.0f\n", carto.x, carto.y);
      printf("o:%.0f %.0f\n", odom.x, odom.y);
      //printf("r:%.0f %.0f\n", robot.x, robot.y);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    rate.sleep();
  }
  return 0;
}
