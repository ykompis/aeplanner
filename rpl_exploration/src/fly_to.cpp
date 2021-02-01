#include <rpl_exploration/FlyToAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace rpl_exploration {
  class FlyTo
  {
    private:
      ros::NodeHandle nh_;
      ros::Publisher pub_;
      actionlib::SimpleActionServer<rpl_exploration::FlyToAction> as_;

      tf::TransformListener listener;
    public:
      FlyTo() : pub_(nh_.advertise<geometry_msgs::PoseStamped>("fly_to_cmd", 1000)),
                as_(nh_, "fly_to", boost::bind(&FlyTo::execute, this, _1, &as_), false)
      {
        ROS_INFO("Starting fly to server");
        as_.start();
      }
      void execute(const rpl_exploration::FlyToGoalConstPtr& goal,
                   actionlib::SimpleActionServer<rpl_exploration::FlyToAction> * as)
      {
        ROS_INFO_STREAM("Got new goal: Fly to (" << goal->pose.pose.position.x << ", "
                                                 << goal->pose.pose.position.y << ", "
                                                 << goal->pose.pose.position.z << ") ");

        double dt = 0.05;
        ros::Rate r(1/dt);

        geometry_msgs::Point p = goal->pose.pose.position;

        float distance_to_goal = 9001; // Distance is over 9000
        float yaw_diff = M_PI;

        tf::StampedTransform transform;
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

        tf::Quaternion gq(
          goal->pose.pose.orientation.x,
          goal->pose.pose.orientation.y,
          goal->pose.pose.orientation.z,
          goal->pose.pose.orientation.w);
        tf::Matrix3x3 m(gq);
        double roll, pitch, goal_yaw;
        m.getRPY(roll, pitch, goal_yaw);

        // interpolate from current position at 20 Hz and comply with max vel and max acc
        constexpr float vel_max = 1.0;     // [m/s]
        constexpr float vel_max_yaw = 1.6; // [rad/s]
        constexpr float acc_max = 1.0;     // [m/s2]
        constexpr float acc_max_yaw = 1.6; // [rad/s2]

        geometry_msgs::PoseStamped current_pose, next_pose;
        float current_yaw;
        float vel = 0;
        float vel_yaw = 0;
        float acc = acc_max;
        float acc_yaw = acc_max_yaw;

        bool current_pose_set = false;
        float yaw_dir;

        // Check if target is reached...
        do
        {
          // ROS_INFO_STREAM("Publishing goal to (" << p.x << ", " << p.y << ", " << p.z << ") ");
          listener.waitForTransform("/world", "/firefly/base_link", ros::Time(0), ros::Duration(10.0) );
          listener.lookupTransform("/world", "/firefly/base_link", ros::Time(0), transform);

          geometry_msgs::Point q;
          q.x = (float)transform.getOrigin().x();
          q.y = (float)transform.getOrigin().y();
          q.z = (float)transform.getOrigin().z();

          geometry_msgs::Quaternion tq;
          tq.x = (float)transform.getRotation().x();
          tq.y = (float)transform.getRotation().y();
          tq.z = (float)transform.getRotation().z();
          tq.w = (float)transform.getRotation().w();
          tf::Quaternion cq( tq.x, tq.y, tq.z, tq.w);
          tf::Matrix3x3 m(cq);
          double actual_yaw;
          m.getRPY(roll, pitch, actual_yaw);

          // proper acceleration, but not bothering with deceleration...
          if(vel < vel_max){
            vel += acc_max * dt;
          }
          if(vel_yaw < vel_max_yaw){
            vel_yaw += acc_max_yaw * dt;
          }

          ROS_INFO_STREAM("Current position: (" << q.x << ", " << q.y << ", " << q.z << ") ");
          geometry_msgs::Point d;
          d.x = p.x - q.x;
          d.y = p.y - q.y;
          d.z = p.z - q.z;

          distance_to_goal = sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
          ROS_INFO_STREAM("Distance to goal: " << distance_to_goal);

          yaw_diff = atan2(sin(goal_yaw-actual_yaw), cos(goal_yaw-actual_yaw));

          if(!current_pose_set){
            current_pose.pose.position = q;
            current_pose.pose.orientation = tq;
            current_yaw = actual_yaw;
            yaw_dir = yaw_diff / fabs(yaw_diff);
            current_pose_set = true;
          }

          tf::Point p_tf, q_tf, current_p_tf;
          tf::pointMsgToTF(p, p_tf);
          tf::pointMsgToTF(q, q_tf);
          tf::pointMsgToTF(current_pose.pose.position, current_p_tf);

          if(distance_to_goal < 0.4){
            vel = 0;
          }
          if(fabs(yaw_diff) < 0.6*M_PI){

          }

          tf::Point dir = 1./distance_to_goal * (p_tf - q_tf);
          tf::Point next_p = current_p_tf + dir * vel * dt;
          geometry_msgs::Point next_p_msg;
          tf::pointTFToMsg(next_p, next_p_msg);
          if(distance_to_goal < 0.4){
            // Send goal position if we're close, but yaw still incorrect
            next_p_msg = goal->pose.pose.position;
          }

          tf::Quaternion next_q;
          float next_yaw = current_yaw + yaw_dir * vel_yaw * dt;
          next_q.setRPY(roll, pitch, next_yaw);
          geometry_msgs::Quaternion next_q_msg;
          tf::quaternionTFToMsg(next_q, next_q_msg);
          if(fabs(yaw_diff) < 0.6*M_PI){
            // Send goal yaw if yaw is close but position still incorrect
            next_q_msg = goal->pose.pose.orientation;
          }

          next_pose.pose.position = next_p_msg;
          next_pose.pose.orientation = next_q_msg;

          pub_.publish(next_pose);

          current_pose = next_pose;
          current_yaw = next_yaw;

          r.sleep();
        } while(distance_to_goal > 0.8 or fabs(yaw_diff) > 0.6*M_PI);

        pub_.publish(goal->pose);

        as->setSucceeded();
      }
  };


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fly_to_server");

  rpl_exploration::FlyTo fly_to;

  ros::spin();
  return 0;
}
