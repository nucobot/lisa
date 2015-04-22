#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// base class for action server
#include <actionlib/server/simple_action_server.h>

// Action description files (check the /action dir)
#include <lisa_move/MoveAction.h>

std::string input_pose_topic;






class ActionServer
{
public:
    ActionServer(ros::NodeHandle nh_) :
        as_move_action  (nh_, "MoveAS",   boost::bind(&ActionServer::moveActionCB,   this, _1), false)
    {
        as_move_action.start();
    }

    ~ActionServer(void) {}



    void moveActionCB(const lisa_move::MoveGoalConstPtr  &goal) {
        lisa_move::MoveResult result_;
        ros::Rate r(60);


        while(true) {
            r.sleep();
            break;
        }

        as_move_action.setSucceeded(result_);
        return;
    }

private:
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<lisa_move::MoveAction> as_move_action;
};









class TfHandle
{
private:
    ros::Subscriber sub_ekf;

    ros::Rate r;


    void get_pose_ekf(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg_in)
    {
        double roll, pitch, yaw;
        tf::Quaternion orient(msg_in->pose.pose.orientation.x, msg_in->pose.pose.orientation.y, msg_in->pose.pose.orientation.z, msg_in->pose.pose.orientation.w);
        orient.normalize();
        tf::Matrix3x3(orient).getRPY(roll, pitch, yaw);
        if (isnan(msg_in->pose.pose.orientation.x) || isnan(msg_in->pose.pose.orientation.y) || isnan(msg_in->pose.pose.orientation.z) || isnan(msg_in->pose.pose.orientation.w)) {
            ROS_WARN("%f, %f, %f, %f", (msg_in->pose.pose.orientation.x), (msg_in->pose.pose.orientation.y), (msg_in->pose.pose.orientation.z), (msg_in->pose.pose.orientation.w));
        }

        if (isnan(roll) || isnan(pitch) || isnan(yaw)) {
            ROS_WARN("RPY failure!!! (%f, %f, %f)", roll, pitch, yaw);
        }
        if (isnan(roll)) {
            roll = 0;
        }
        if (isnan(pitch)) {
            pitch = 0;
        }
        if (isnan(yaw)) {
            yaw = 0;
        }
    }

    void spin()
    {
        ros::spinOnce();
        this->r.sleep();
    }

public:
    TfHandle(ros::NodeHandle nh): r(20)
    {
        this->sub_ekf = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> (input_pose_topic, 1, &TfHandle::get_pose_ekf, this);
        while (ros::ok()) {
            this->spin();
        }
    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "lisa_move");
    ros::NodeHandle nh;

    if (!nh.getParam("lisa_move/input_pose_topic", input_pose_topic)) input_pose_topic = "/robot_pose_ekf/odom";

    TfHandle tf_handle(nh);
    ActionServer action_server(nh);

    ros::spin();
    return 0;
};
