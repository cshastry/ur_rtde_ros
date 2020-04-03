#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <ur_control_msgs/MoveJoint.h>
#include <ur_control_msgs/MovePose.h>
#include <ur_control_msgs/ServoJoint.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <Eigen/Geometry>

#include <thread>

auto convert(const Eigen::Vector3d& v)
{
    geometry_msgs::Point m;
    m.x = v.x();
    m.y = v.y();
    m.z = v.z();
    return m;
}
auto convert(const Eigen::Quaterniond& q)
{
    geometry_msgs::Quaternion m;
    m.w = q.w();
    m.x = q.x();
    m.y = q.y();
    m.z = q.z();
    return m;
}

auto convert(const std::vector<double>& pose)
{
    assert(pose.size() == 6);
    Eigen::Vector3d p(pose[0], pose[1], pose[2]); // position
    Eigen::Vector3d r(pose[3], pose[4], pose[5]); // rotation
    Eigen::Quaterniond q(Eigen::AngleAxisd(r.norm(), r.normalized()));
    return std::make_tuple(convert(p), convert(q));
}

auto convert(const geometry_msgs::Pose& m)
{
    Eigen::AngleAxisd aa(Eigen::Quaterniond(m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z));
    Eigen::Vector3d r = aa.axis() * aa.angle();
    return std::vector<double>{m.position.x, m.position.y, m.position.z, r[0], r[1], r[2]};
}

auto convert(const geometry_msgs::Twist& m)
{
    return std::vector<double>{m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z};
}

class ReceiveInterface : public ur_rtde::RTDEReceiveInterface
{
public:
    using ur_rtde::RTDEReceiveInterface::RTDEReceiveInterface;

    sensor_msgs::JointState getJointStateMsg()
    {
        sensor_msgs::JointState m;
        m.position = getActualQ();
        m.velocity = getActualQd();
        // m.effort =
        m.header.stamp = ros::Time::now();
        return m;
    }

    geometry_msgs::PoseStamped getActualTCPPoseStampedMsg()
    {
        geometry_msgs::PoseStamped m;
        // m.header.frame_id = ""; FIXME
        std::tie(m.pose.position, m.pose.orientation) = convert(getActualTCPPose());
        m.header.stamp = ros::Time::now();
        return m;
    }
};

class ControlInterface : public ur_rtde::RTDEControlInterface
{
public:
    using ur_rtde::RTDEControlInterface::RTDEControlInterface;

    void moveJ(const ur_control_msgs::MoveJoint& m)
    {
        double speed = (m.speed != 0.0) ? m.speed : 1.05;
        double acceleration = (m.acceleration != 0.0) ? m.acceleration : 1.4;
        ur_rtde::RTDEControlInterface::moveJ(m.position, speed, acceleration);
    }

    void moveL(const ur_control_msgs::MovePose& m)
    {
        double speed = (m.speed != 0.0) ? m.speed : 0.25;
        double acceleration = (m.acceleration != 0.0) ? m.acceleration : 1.2;
        ur_rtde::RTDEControlInterface::moveL(convert(m.pose), speed, acceleration);
    }

    void servoJ(const ur_control_msgs::ServoJoint& m)
    {
        double time = (m.time != 0.0) ? m.time : 0.002;
        double lookahead_time = (m.lookahead_time != 0.0) ? m.lookahead_time : 0.1;
        double gain = (m.gain != 0.0) ? m.gain : 300;
        ur_rtde::RTDEControlInterface::servoJ(m.position, 0.0, 0.0, time, lookahead_time, gain);
    }
};

int main(int argc, char* argv[])
{
    using namespace std::string_literals;

    ros::init(argc, argv, "ur_control");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::NodeHandle nh_servo;

    // Servo callbacks are blocking, so we use a dedicated callback queue and
    // process its callbacks from a separate thread
    ros::CallbackQueue servo_queue;
    nh_servo.setCallbackQueue(&servo_queue);

    auto publish_tcp_pose = nh_priv.param("publish_tcp_pose", false);
    auto robot_ip = nh_priv.param("robot_ip", "192.168.10.201"s);

    ReceiveInterface ur_receiver(robot_ip);
    ControlInterface ur_controller(robot_ip);

    std::list<ros::Subscriber> subscribers{
        nh.subscribe("move_j", 1, &ControlInterface::moveJ, &ur_controller),
        nh_servo.subscribe("servo_j", 10, &ControlInterface::servoJ, &ur_controller),
    };

    auto pub_joint_state = nh.advertise<sensor_msgs::JointState>("joint_state", 1);
    auto pub_tcp_pose = (publish_tcp_pose) ? nh.advertise<geometry_msgs::PoseStamped>("tcp_pose", 1) : ros::Publisher();

    std::thread servo_thread([&] {
        bool servoing = false;

        while (nh_servo.ok()) {
            auto ret = servo_queue.callOne(ros::WallDuration(0.1));

            if ((ret == ros::CallbackQueue::Called) && !servoing) {
                // If a callback got called the robot is now servoing
                servoing = true;
            } else if ((ret == ros::CallbackQueue::Empty) && servoing) {
                // Stop servoing if there are no more servo commands left in
                // the queue after timing out
                ur_controller.servoStop();
                servoing = false;
            }
        }

        ur_controller.servoStop();
    });

    // Schedule timer to publish robot state
    auto timer = nh.createTimer(ros::Duration(ur_controller.getStepTime()),
                                [&](const auto&) {
                                    pub_joint_state.publish(ur_receiver.getJointStateMsg());

                                    if (publish_tcp_pose)
                                        pub_tcp_pose.publish(ur_receiver.getActualTCPPoseStampedMsg());
                                });

    ros::spin();

    servo_thread.join();
    ur_controller.stopRobot();

    return EXIT_SUCCESS;
}
