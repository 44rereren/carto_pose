#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32MultiArray.h>
#include <sys/time.h>

#define LIDAR_DIST_X 0.39171866199f //Lidarとロボット中心のX軸距離[m]
#define LIDAR_DIST_Y 0.f //Lidarとロボット中心のY軸距離[m]
#define INIT_X 4563.f//4547.395f//ロボットの初期座標[mm]
#define INIT_Y -9516.f //9381.325f //ロボットの初期座標[mm]
#define FREQUENCY 1000 //[Hz]
#define N 20

typedef struct
{
  float x;
  float y;
  float theta;
  int seg;
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

float weightAverageFilter(float odme, float carto)
{
	float gain;
	if(fabs(odme - carto) > 60)
	{
		gain = 1.f;
	}
	else if(fabs(odme - carto) <= 40)
	{
		gain = 0.5f;
	}
	else if(fabs(odme - carto) <= 60)
	{
		gain = 0.7f;
	}
  return gain * odme + (1.f - gain) * carto;
}

void callback(const std_msgs::Float32MultiArray &msg)
{
  odom.x = msg.data[0]; //[mm]
  odom.y = msg.data[1]; //[mm]
  odom.theta = msg.data[2]; //[rad]  default: cw -> +, ccw -> -
  odom.seg = msg.data[3];
}

static struct timeval microsSource;
static int64_t offsetSeconds;
static int64_t offsetMicros;

void timer_setup(){
    gettimeofday(&microsSource, NULL);
    offsetSeconds=microsSource.tv_sec;
    offsetMicros=microsSource.tv_usec;
    return;
}

int64_t micros() {
    gettimeofday(&microsSource, NULL);
    return (microsSource.tv_sec-offsetSeconds)*(int64_t)1000000+(microsSource.tv_usec-offsetMicros);
}

int64_t millis(){
	return micros() * 0.001;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_pose");
  ros::NodeHandle nh;
  ros::Rate rate(FREQUENCY);
  tf::TransformListener listener;
  ros::Subscriber stm_sub = nh.subscribe("stm_data", 1000, callback);
  ros::Publisher robot_pub = nh.advertise<std_msgs::Float32MultiArray>("robot_pose",100);

  float sum_x = 0.f;
  float sum_y = 0.f;
  int sum_n = 0;
  float ave_x = 0.f;
  float ave_y = 0.f;

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
      
      //
      sum_x += carto.x;
      sum_y += carto.y;
      sum_n++;
      if(sum_n == N)
      {
      	ave_x = sum_x / float(N);
      	ave_y = sum_y / float(N);
        robot.theta = odom.theta; //[rad]
        robot.x = weightAverageFilter(odom.x, ave_x); //[mm]
        robot.y = weightAverageFilter(odom.y, ave_y); //[mm]
        ROS_INFO("r:%.0f %.0f a:%.0f %.0f o:%.0f %.0f yaw:%.3f seg:%d", robot.x, robot.y, ave_x, ave_y, odom.x, odom.y, odom.theta * 180.f / M_PI, odom.seg);
      	robot_array.data[0] = robot.x;
      	robot_array.data[1] = robot.y;
      	robot_pub.publish(robot_array);
      	sum_x = sum_y = 0.f;
      	sum_n = 0;
      }
      //
      
      /*robot_array.data[0] = carto.x;
      robot_array.data[1] = carto.y;
      robot_pub.publish(robot_array);*/

      //printf("o:%.0f %.0f\n", odom.x, odom.y);
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
