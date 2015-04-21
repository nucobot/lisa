#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <boost/assign/list_of.hpp> // for 'list_of()

#define IMU_ANG_VX 0  // IMU: imu_msg.angular_velocity.x
#define IMU_ANG_VY 1  // IMU: imu_msg.angular_velocity.y
#define IMU_ANG_VZ 2  // IMU: imu_msg.angular_velocity.z
#define IMU_LIN_AX 3  // IMU: imu_msg.linear_acceleration.x
#define IMU_LIN_AY 4  // IMU: imu_msg.linear_acceleration.y
#define IMU_LIN_AZ 5  // IMU: imu_msg.linear_acceleration.z
#define MAG_SCAL_X 6  // Magnetometer scaled.XAxis
#define MAG_SCAL_Y 7  // Magnetometer scaled.YAxis
#define MAG_SCAL_Z 8  // Magnetometer scaled.ZAxis
#define BAT_VOLT_1 9  // Battery voltage on BAT1_PIN
#define BAT_VOLT_2 10 // Battery voltage on BAT2_PIN
#define ENC_DATA_0 11 // Speed on encoder 0
#define ENC_DATA_1 12 // Speed on encoder 1
#define BTN_STATE  13 // Power button state
#define DATA_SIZE  14 // Overall array size

ros::Publisher  odom_pub;
ros::Publisher  imu_pub;
ros::Publisher  mag_pub;
ros::Subscriber apm_sub;

ros::Time current_time, last_time;
#define MAX_DELTA_TIME 0.1

double x = 0.0;
double y = 0.0;
double yaw = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

double ax = 0.0;
double ay = 0.0;

double raw_to_meter = 100; // 1 meter per second = raw_to_meter of raw data
double separation = 0.175; // separation of sensors in meters

void apmCallback(const std_msgs::Float32MultiArray::ConstPtr& apm) {
    // Filling out the velocities
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    if (fabs(dt) > MAX_DELTA_TIME) {
        last_time = current_time;
        return; // This is for the first iteration when 'last_time' is undefined
    }

    double lx = dt * double(apm->data[ENC_DATA_0]) / raw_to_meter;
    double rx = dt * double(apm->data[ENC_DATA_1]) / raw_to_meter;

    double dyaw = (lx - rx) / separation;
    double dx = rx, dy = 0;
    if (dyaw != 0) {
    	dx = sin(dyaw) * (separation/2 + rx/dyaw);
    	dy = (1 - cos(dyaw)) * (separation/2 + rx/dyaw);
    }

    double delta_x = dx * cos(yaw) - dy * sin(yaw);
    double delta_y = dx * sin(yaw) + dy * cos(yaw);
    x += delta_x;
    y += delta_y;
    yaw += dyaw;

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "encoder_link";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = delta_x / dt;
    odom.twist.twist.linear.y = delta_y / dt;
    odom.twist.twist.angular.z = dyaw / dt;

    //FIXME: insert normal values
    odom.pose.covariance =  boost::assign::list_of(1e-3)  (0) (0)  (0)  (0)  (0)
                                                  (0) (1e-3)  (0)  (0)  (0)  (0)
                                                  (0)   (0)  (1e6) (0)  (0)  (0)
                                                  (0)   (0)   (0) (1e6) (0)  (0)
                                                  (0)   (0)   (0)  (0) (1e6) (0)
                                                  (0)   (0)   (0)  (0)  (0)  (1e3);

    odom.twist.covariance = boost::assign::list_of(1e-3)  (0) (0)  (0)  (0)  (0)
                                                  (0) (1e-3)  (0)  (0)  (0)  (0)
                                                  (0)   (0)  (1e6) (0)  (0)  (0)
                                                  (0)   (0)   (0) (1e6) (0)  (0)
                                                  (0)   (0)   (0)  (0) (1e6) (0)
                                                  (0)   (0)   (0)  (0)  (0)  (1e3);
    //publish the message
    odom_pub.publish(odom);


    //next, we'll publish the imu message over ROS
    sensor_msgs::Imu imu;
    imu.header.stamp = current_time;
    imu.header.frame_id = "apm_link";

    geometry_msgs::Quaternion imu_quat = tf::createQuaternionMsgFromYaw(apm->data[MAG_SCAL_Z]);
    imu.orientation = imu_quat;
    imu.angular_velocity.x = apm->data[IMU_ANG_VX];
    imu.angular_velocity.y = apm->data[IMU_ANG_VY];
    imu.angular_velocity.z = apm->data[IMU_ANG_VZ];

    imu.linear_acceleration.x = apm->data[IMU_LIN_AX]; 
    imu.linear_acceleration.y = apm->data[IMU_LIN_AY];
    imu.linear_acceleration.z = apm->data[IMU_LIN_AZ];


    //FIXME: insert normal values
    imu.orientation_covariance = boost::assign::list_of (1e-3)  (0) (0)
                                                        (0) (1e-3)  (0)
                                                        (0)   (0)  (1e6);

    imu.angular_velocity_covariance = boost::assign::list_of (1e-3)  (0) (0)
                                                             (0) (1e-3)  (0)
                                                             (0)   (0)  (1e6);

    imu.linear_acceleration_covariance = boost::assign::list_of (1e-3)  (0) (0)
                                                                (0) (1e-3)  (0)
                                                                (0)   (0)  (1e6);

    //publish the message
    imu_pub.publish(imu);

    //next, we'll publish the magnetometer message over ROS
    geometry_msgs::Vector3Stamped mag;
    mag.header.stamp = current_time;
    mag.header.frame_id = "apm_frame";

    mag.vector.x = apm->data[MAG_SCAL_X];
    mag.vector.y = apm->data[MAG_SCAL_Y];
    mag.vector.z = apm->data[MAG_SCAL_Z];

    //publish the message
    mag_pub.publish(mag);

    last_time = current_time;

    ROS_INFO("dt: %1.6f, (%3.3f, %3.3f) | (%3.3f, %3.3f) | (%3.3f, %3.3f)", dt, x, y, vx, vy, vth, th);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lisa_apm_driver");
    ros::NodeHandle nh;

    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
    imu_pub  = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 50);
    mag_pub  = nh.advertise<geometry_msgs::Vector3Stamped>("imu/mag", 50);
    apm_sub  = nh.subscribe("/apm/data_raw", 50, apmCallback);

    current_time = ros::Time::now();
    last_time    = ros::Time::now();

    ros::spin();
    return 0;
}
