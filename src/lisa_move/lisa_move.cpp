#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// base class for action server
#include <actionlib/server/simple_action_server.h>

// Action description files (check the /action dir)
#include <lisa_move/MoveAction.h>

std::string world_frame;
std::string base_link_frame;


class ActionServer
{
private:
    tf::StampedTransform transform;

public:
    ActionServer(ros::NodeHandle nh_) :
        as_move_action  (nh_, "MoveAS",   boost::bind(&ActionServer::moveActionCB,   this, _1), false)
    {
        as_move_action.start();

        try {
            this->tf_listener.waitForTransform(base_link_frame, world_frame, ros::Time(0), ros::Duration(10.0) );
            this->tf_listener.lookupTransform(base_link_frame, world_frame, ros::Time(0), this->transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }
    }

    ~ActionServer() {}

    void moveActionCB(const lisa_move::MoveGoalConstPtr &goal) {
        lisa_move::MoveResult result_;
        ros::Rate r(60);

        while(true) {
            try {
                this->tf_listener.lookupTransform(base_link_frame, world_frame, ros::Time(0), this->transform);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                r.sleep();
                continue;
            }
            
            double sx = goal->x - this->tf_listener.getOrigin().x();
            double sy = goal->y - this->tf_listener.getOrigin().y();




            r.sleep();
            break;
        }

        as_move_action.setSucceeded(result_);
        return;
    }

private:
    tf::TransformListener tf_listener;
    actionlib::SimpleActionServer<lisa_move::MoveAction> as_move_action;
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "lisa_move");
    ros::NodeHandle nh;

    if (!nh.getParam("lisa_move/world_frame", world_frame)) {ROS_ERROR("Failed to get param 'lisa_move/world_frame'"); ros::shutdown(); };
    if (!nh.getParam("lisa_move/base_link_frame", base_link_frame)) {ROS_ERROR("Failed to get param 'lisa_move/base_link_frame'"); ros::shutdown(); };

    ActionServer action_server(nh);

    ros::spin();
    return 0;
};
