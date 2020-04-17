#include "monotonic.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <ros/ros.h>

#include <Eigen/Geometry>

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

using namespace std::chrono_literals;

auto convertPoint(const Eigen::Vector3d& v)
{
    geometry_msgs::Point m;
    m.x = v.x();
    m.y = v.y();
    m.z = v.z();
    return m;
}

auto convertQuaternion(const Eigen::Quaterniond& q)
{
    geometry_msgs::Quaternion m;
    m.w = q.w();
    m.x = q.x();
    m.y = q.y();
    m.z = q.z();
    return m;
}

auto convertPose(const std::vector<double>& pose)
{
    assert(pose.size() == 6);
    Eigen::Vector3d p(pose[0], pose[1], pose[2]); // position
    Eigen::Vector3d r(pose[3], pose[4], pose[5]); // rotation
    Eigen::Quaterniond q(Eigen::AngleAxisd(r.norm(), r.normalized()));
    return std::make_tuple(convertPoint(p), convertQuaternion(q));
}

auto convertPose(const geometry_msgs::Pose& m)
{
    Eigen::AngleAxisd aa(Eigen::Quaterniond(m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z));
    Eigen::Vector3d r = aa.axis() * aa.angle();
    return std::vector<double>{m.position.x, m.position.y, m.position.z, r[0], r[1], r[2]};
}

auto convertTwist(const std::vector<double>& twist)
{
    assert(twist.size() == 6);
    geometry_msgs::Twist m;
    m.linear.x = twist[0];
    m.linear.y = twist[1];
    m.linear.z = twist[2];
    m.angular.x = twist[3];
    m.angular.y = twist[4];
    m.angular.z = twist[5];
    return m;
}

auto convertTwist(const geometry_msgs::Twist& m)
{
    return std::vector<double>{m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z};
}

// Wrapper around ur_rtde::RTDEReceiveInterface to bridge with ROS message types.
class Receiver
{
public:
    explicit Receiver(std::string base_frame,
                      std::string tool_frame,
                      std::string prefix,
                      std::string hostname,
                      std::vector<std::string> variables,
                      int port = 30004)
        : rtde_recv_(std::move(hostname), std::move(variables), port)
        , base_frame_(prefix + base_frame)
        , tool_frame_(prefix + tool_frame)
    {
        static const std::vector<std::string> base_joint_names = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        };

        for (const auto& n : base_joint_names)
            joint_names_.push_back(prefix + n);
    }

    sensor_msgs::JointState getJointStateMsg()
    {
        sensor_msgs::JointState m;
        m.header.stamp = ros::Time::now();
        m.name = joint_names_;
        m.position = rtde_recv_.getActualQ();
        m.velocity = rtde_recv_.getActualQd();
        m.effort = rtde_recv_.getActualCurrent();
        return m;
    }

    geometry_msgs::PoseStamped getActualTCPPoseMsg()
    {
        geometry_msgs::PoseStamped m;
        m.header.stamp = ros::Time::now();
        m.header.frame_id = base_frame_;
        std::tie(m.pose.position, m.pose.orientation) = convertPose(rtde_recv_.getActualTCPPose());
        return m;
    }

    geometry_msgs::TwistStamped getActualTCPTwistMsg()
    {
        geometry_msgs::TwistStamped m;
        m.header.stamp = ros::Time::now();
        m.header.frame_id = tool_frame_;
        m.twist = convertTwist(rtde_recv_.getActualTCPSpeed());
        return m;
    }

private:
    ur_rtde::RTDEReceiveInterface rtde_recv_;
    std::string base_frame_;
    std::string tool_frame_;
    std::vector<std::string> joint_names_;
};

// Wrapper around ur_rtde::RTDEControlInterface to bridge with ROS message types.
//
// Maintains its own thread for processing robot commands because
//  1) the MoveX commands are blocking and we don't want them to block the ROS
//     event loop; and
//  2) the ServoX commands needs to be sent periodically, thus also requiring
//     control of the timing between calls.
class Controller
{
    enum State
    {
        IDLE,
        MOVING,
        SERVOING,
    };

public:
    explicit Controller(std::string hostname, int port = 30004)
        : rtde_ctrl_(std::move(hostname), port)
        , step_time_(rtde_ctrl_.getStepTime())
        , servo_timeout_duration_(100ms)
        , state_(IDLE)
        , cmd_proc_stopped_(true)
        , rate_(step_time_)
    {
    }

    ~Controller()
    {
        stop();
        rtde_ctrl_.stopScript();
    }

    void start()
    {
        if (thread_.joinable()) {
            ROS_WARN("Servo loop already running?");
            return;
        }

        cmd_proc_stopped_ = false;
        thread_ = std::thread([this]() { processCommands(); });
    }

    void stop()
    {
        cmd_proc_stopped_ = true;

        if (thread_.joinable())
            thread_.join();
    }

    double getStepTime() const
    {
        return step_time_.count();
    }

    void moveJ(sensor_msgs::JointState m)
    {
        if (state_ == SERVOING) {
            ROS_WARN("Discarding MoveJ command - currently servoing");
            return;
        }

        enqueueCommand([this, m = std::move(m)]() {
            state_ = MOVING;

            // blocks until robot is done moving
            if (!rtde_ctrl_.moveJ(m.position, 1.05, 1.4))
                ROS_WARN("MoveJ command failed");

            state_ = IDLE;
        });
    }

    void moveL(geometry_msgs::PoseStamped m)
    {
        if (state_ == SERVOING) {
            ROS_WARN("Discarding MoveL command - currently servoing");
            return;
        }

        enqueueCommand([this, m = std::move(m)]() {
            state_ = MOVING;

            // blocks until robot is done moving
            if (!rtde_ctrl_.moveL(convertPose(m.pose), 0.25, 1.2))
                ROS_WARN("MoveL command failed");

            state_ = IDLE;
        });
    }

    void servoJ(sensor_msgs::JointState m)
    {
        if (state_ == MOVING) {
            ROS_WARN("Discarding servoJ command - currently moving");
            return;
        }

        enqueueCommand([this, m = std::move(m)]() {
            if (state_ != SERVOING) {
                rate_.reset();
                state_ = SERVOING; // cleared if we don't keep receiving servo commands for some time
            }

            // servoJ is non-blocking
            if (!rtde_ctrl_.servoJ(m.position, 0.0, 0.0, step_time_.count(), 0.1, 300))
                ROS_WARN("ServoJ command failed");

            if (!rate_.sleep())
                ROS_WARN("ServoJ loop rate not met");
        });
    }

private:
    template <typename Callable>
    void enqueueCommand(Callable&& f)
    {
        {
            std::lock_guard<std::mutex> lock(cmd_queue_mtx_);
            cmd_queue_.emplace(std::forward<Callable>(f));
        }

        cmd_cv_.notify_one();
    }

    // Process servo commands from the queue in a loop
    void processCommands()
    {
        while (!cmd_proc_stopped_) {
            std::unique_lock<std::mutex> lock(cmd_queue_mtx_);

            // Wait for queue to be non-empty or stop, but time out after the
            // given period of time
            bool timeout = !cmd_cv_.wait_for(lock,
                                             servo_timeout_duration_,
                                             [this]() { return !cmd_queue_.empty() || cmd_proc_stopped_; });

            if (cmd_proc_stopped_)
                break;

            if (timeout) {
                // Clear SERVOING state after timeout (ie. no new servo
                // messages were received after a period of time)
                if (state_ == SERVOING) {
                    rtde_ctrl_.servoStop();
                    state_ = IDLE;
                }
            } else {
                auto cmd = std::move(cmd_queue_.front());
                cmd_queue_.pop();
                lock.unlock(); // unlock before the executing command
                cmd();
            }
        }

        if (state_ == SERVOING)
            rtde_ctrl_.servoStop();

        state_ = IDLE;
    }

private:
    ur_rtde::RTDEControlInterface rtde_ctrl_;
    std::chrono::duration<double> step_time_;
    std::chrono::duration<double> servo_timeout_duration_;
    std::atomic<State> state_;
    std::atomic<bool> cmd_proc_stopped_;
    std::condition_variable cmd_cv_;
    std::mutex cmd_queue_mtx_;
    std::queue<std::function<void()>> cmd_queue_;
    std::thread thread_;
    monotonic_rate rate_;
};

int main(int argc, char* argv[])
{
    using namespace std::string_literals;

    ros::init(argc, argv, "ur_control");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    auto publish_tcp_pose = nh_priv.param("publish_tcp_pose", false);
    auto publish_tcp_twist = nh_priv.param("publish_tcp_twist", false);
    auto prefix = nh_priv.param("prefix", ""s);
    auto hostname = nh_priv.param("hostname", ""s);
    auto port = nh_priv.param("port", 30004);

    if (hostname.empty())
        throw std::runtime_error("The ~hostname parameter is not set");

    std::vector<std::string> rtde_output_variables = {"actual_q", "actual_qd", "actual_current"};

    if (publish_tcp_pose)
        rtde_output_variables.push_back("actual_TCP_pose");

    if (publish_tcp_twist)
        rtde_output_variables.push_back("actual_TCP_speed");

    Receiver receiver("base", "tcp", prefix, hostname, rtde_output_variables, port);
    Controller controller(hostname, port);

    auto pub_joint_state = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    auto pub_tcp_pose = (publish_tcp_pose) ? nh.advertise<geometry_msgs::PoseStamped>("tcp_pose", 1) : ros::Publisher{};
    auto pub_tcp_twist = (publish_tcp_twist) ? nh.advertise<geometry_msgs::TwistStamped>("tcp_twist", 1) : ros::Publisher{};

    std::list<ros::Subscriber> subscribers{
        nh.subscribe("move_j", 2, &Controller::moveJ, &controller, ros::TransportHints().tcpNoDelay()),
        nh.subscribe("move_l", 2, &Controller::moveL, &controller, ros::TransportHints().tcpNoDelay()),
        nh.subscribe("servo_j", 8, &Controller::servoJ, &controller, ros::TransportHints().tcpNoDelay()),
    };

    // Schedule timer to publish robot state
    auto timer = nh.createSteadyTimer(ros::WallDuration(controller.getStepTime()),
                                      [&](const auto&) {
                                          pub_joint_state.publish(receiver.getJointStateMsg());

                                          if (publish_tcp_pose)
                                              pub_tcp_pose.publish(receiver.getActualTCPPoseMsg());

                                          if (publish_tcp_twist)
                                              pub_tcp_pose.publish(receiver.getActualTCPTwistMsg());
                                      });
    controller.start();
    ros::spin();
    controller.stop();

    return EXIT_SUCCESS;
}
