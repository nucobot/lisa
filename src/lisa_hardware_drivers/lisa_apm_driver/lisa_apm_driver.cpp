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

ros::Subscriber apm_sub;
ros::Time current_time, last_time;


double raw_to_meter; // 1 meter per second = raw_to_meter of raw data
double separation;   // separation of sensors in meters
std::string odom_topic = "lisa_apm_driver/odom";
std::string imu_topic  = "lisa_apm_driver/imu";
std::string odom_frame = "encoder_link";
std::string imu_frame  = "apm_link";

class Odometry {
private:
    ros::NodeHandle nh;
    double separation;
    double raw_to_meter;
    ros::Publisher odom_pub;
    nav_msgs::Odometry odom_msg;

public:
    double x, y, yaw; // position
    double vx, vy, w; // velocity

public:
    Odometry(ros::NodeHandle &nh_, double sep_, double to_m_) : nh(nh_) {
        this->x = this->y = this->yaw = 0.0;
        this->vx = this->vy = this->w = 0.0;
        this->separation = sep_;
        this->raw_to_meter = to_m_;
        this->odom_pub = this->nh.advertise<nav_msgs::Odometry>(odom_topic, 10);
    }

    void spin_once(double dt, double enc0, double enc1) {
        double lx = dt * enc0 / this->raw_to_meter;
        double rx = dt * enc1 / this->raw_to_meter;

        double dyaw = (lx - rx) / this->separation;
        double dx = rx, dy = 0;
        if (dyaw != 0) {
    	    dx = sin(dyaw) * (this->separation/2 + rx/dyaw);
    	    dy = (1 - cos(dyaw)) * (this->separation/2 + rx/dyaw);
        }

        double delta_x = dx * cos(this->yaw) - dy * sin(this->yaw);
        double delta_y = dx * sin(this->yaw) + dy * cos(this->yaw);
        this->x += delta_x;
        this->y += delta_y;
        this->yaw += dyaw;

        this->vx =  delta_x / dt;
        this->vy =  delta_y / dt;
        this->w  =  dyaw / dt;
    }

    void send(ros::Time current_time) {
        this->odom_msg.header.stamp = current_time;
        this->odom_msg.header.frame_id = odom_frame;

        // set the position
        this->odom_msg.pose.pose.position.x = this->x;
        this->odom_msg.pose.pose.position.y = this->y;
        this->odom_msg.pose.pose.position.z = 0;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(this->yaw);
        this->odom_msg.pose.pose.orientation = odom_quat;

        // set the velocity
        this->odom_msg.twist.twist.linear.x = this->vx;
        this->odom_msg.twist.twist.linear.y = this->vy;
        this->odom_msg.twist.twist.angular.z = this->w;

        // FIXME: check the values
        this->odom_msg.pose.covariance =  boost::assign::list_of(1e-2)  (0) (0)  (0)  (0)  (0)
                                                                (0) (1e-2)  (0)  (0)  (0)  (0)
                                                                (0)   (0)  (1e6) (0)  (0)  (0)
                                                                (0)   (0)   (0) (1e6) (0)  (0)
                                                                (0)   (0)   (0)  (0) (1e6) (0)
                                                                (0)   (0)   (0)  (0)  (0)  (1e-1);

        this->odom_msg.twist.covariance = boost::assign::list_of(1e-2)  (0) (0)  (0)  (0)  (0)
                                                                (0) (1e-2)  (0)  (0)  (0)  (0)
                                                                (0)   (0)  (1e6) (0)  (0)  (0)
                                                                (0)   (0)   (0) (1e6) (0)  (0)
                                                                (0)   (0)   (0)  (0) (1e6) (0)
                                                                (0)   (0)   (0)  (0)  (0)  (1e-1);
        // publish the message 
        odom_pub.publish(this->odom_msg);
    }
};


class Imu {
private:
    ros::NodeHandle nh;
    ros::Publisher imu_pub;
    sensor_msgs::Imu imu_msg;
    double ang[3];
    double lin[3];
    double mag[3];

public:
    Imu(ros::NodeHandle &nh_) : nh(nh_) {
        this->imu_pub  = this->nh.advertise<sensor_msgs::Imu>(imu_topic, 10);
    }

    void spin_once(double dt, double ax, double ay, double az,
                                double lx, double ly, double lz,
                                double mx, double my, double mz) {
        this->ang[0] = ax;
        this->ang[1] = ay;
        this->ang[2] = az;

        this->lin[0] = lx;
        this->lin[1] = ly;
        this->lin[2] = lz;

        this->mag[0] = mx;
        this->mag[1] = my;
        this->mag[2] = mz;
    }


    void send(ros::Time current_time) {
        this->imu_msg.header.stamp = current_time;
        this->imu_msg.header.frame_id = imu_frame;

        geometry_msgs::Quaternion imu_quat = tf::createQuaternionMsgFromYaw(this->mag[2]);
        this->imu_msg.orientation = imu_quat;
        this->imu_msg.angular_velocity.x = this->ang[0];
        this->imu_msg.angular_velocity.y = this->ang[1];
        this->imu_msg.angular_velocity.z = this->ang[2];

        this->imu_msg.linear_acceleration.x = this->lin[0];
        this->imu_msg.linear_acceleration.y = this->lin[1];
        this->imu_msg.linear_acceleration.z = this->lin[2];


        // FIXME: check the values
        this->imu_msg.orientation_covariance = boost::assign::list_of (1e6)  (0) (0)
                                                                      (0) (1e6)  (0)
                                                                      (0)   (0)  (1e6);

        this->imu_msg.angular_velocity_covariance = boost::assign::list_of (1e-1)  (0) (0)
                                                                           (0) (1e-1)  (0)
                                                                           (0)   (0)  (1e-1);

        this->imu_msg.linear_acceleration_covariance = boost::assign::list_of (1e-3)  (0) (0)
                                                                              (0) (1e-3)  (0)
                                                                              (0)   (0)  (1e-3);
        // publish the message
        imu_pub.publish(this->imu_msg);
    }
};


boost::shared_ptr<Odometry> odom_handler;
boost::shared_ptr<Imu>      imu_handler;


void apmCallback(const std_msgs::Float32MultiArray::ConstPtr& apm) {
    // Filling out the velocities
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    if (fabs(dt) > 0.1) {
        last_time = current_time;
        return; // This is for the first iteration when 'last_time' is undefined
    }

    odom_handler->spin_once(dt, apm->data[ENC_DATA_0], apm->data[ENC_DATA_1]);
    imu_handler->spin_once(dt, apm->data[IMU_ANG_VX], apm->data[IMU_ANG_VY], apm->data[IMU_ANG_VZ],
                               apm->data[IMU_LIN_AX], apm->data[IMU_LIN_AY], apm->data[IMU_LIN_AZ],
                               apm->data[MAG_SCAL_X], apm->data[MAG_SCAL_Y], apm->data[MAG_SCAL_Z]);

    odom_handler->send(current_time);
    imu_handler->send(current_time);

    last_time = current_time;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lisa_apm_driver");
    ros::NodeHandle nh;

    if (!nh.getParam("lisa_apm_driver/wheel_separation", separation)) {ROS_ERROR("Failed to get param 'lisa_apm_driver/separation'"); ros::shutdown(); };
    if (!nh.getParam("lisa_apm_driver/raw_to_meter", raw_to_meter)) {ROS_ERROR("Failed to get param 'lisa_apm_driver/raw_to_meter'"); ros::shutdown(); };
    if (!nh.getParam("lisa_apm_driver/odom_frame", odom_frame)) ROS_WARN("Failed to get param 'lisa_apm_driver/odom_frame'. Default: %s", odom_frame.c_str());
    if (!nh.getParam("lisa_apm_driver/imu_frame", imu_frame)) ROS_WARN("Failed to get param 'lisa_apm_driver/apm_link'. Default: %s", imu_frame.c_str());

    odom_handler = boost::shared_ptr<Odometry> (new Odometry(nh, separation, raw_to_meter));
    imu_handler  = boost::shared_ptr<Imu> (new Imu(nh));

    apm_sub  = nh.subscribe("/apm/data_raw", 50, apmCallback);

    current_time = ros::Time::now();
    last_time    = ros::Time::now();

    ros::spin();
    return 0;
}
