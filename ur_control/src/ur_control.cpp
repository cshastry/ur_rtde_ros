#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <ur_control_msgs/MoveJoint.h>
#include <ur_control_msgs/MovePose.h>
#include <ur_control_msgs/ServoJoint.h>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <Eigen/Geometry>

#include <condition_variable>
#include <mutex>
#include <queue>
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

class Receiver
{
public:
    explicit Receiver(std::string base_frame,
                      // std::string tool_frame,
                      std::string prefix,
                      std::string hostname,
                      int port = 30004)
        : rtde_recv_(std::move(hostname), {"actual_q", "actual_qd", "actual_TCP_pose"}, port)
        , base_frame_(prefix + base_frame)
    {
        static const std::vector<std::string> base_joint_names{
            "base_joint",
            "shoulder_joint",
            "elbow_joint",
            "wrist1_joint",
            "wrist2_joint",
            "wrist3_joint",
        };

        for (const auto& n : base_joint_names)
            joint_names_.push_back(prefix + n);
    }

    sensor_msgs::JointState getJointStateMsg()
    {
        sensor_msgs::JointState m;
        m.name = joint_names_;
        m.position = rtde_recv_.getActualQ();
        m.velocity = rtde_recv_.getActualQd();
        // TODO m.effort
        m.header.stamp = ros::Time::now();
        return m;
    }

    geometry_msgs::PoseStamped getActualTCPPoseStampedMsg()
    {
        geometry_msgs::PoseStamped m;
        m.header.frame_id = base_frame_;
        std::tie(m.pose.position, m.pose.orientation) = convert(rtde_recv_.getActualTCPPose());
        m.header.stamp = ros::Time::now();
        return m;
    }

private:
    ur_rtde::RTDEReceiveInterface rtde_recv_;
    std::string base_frame_;
    //std::string tool_frame_;
    std::vector<std::string> joint_names_;
};

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
        , servo_timeout_duration_(10 * step_time_)
        , state_(IDLE)
        , cmd_proc_stopped_(true)
    {
    }

    ~Controller()
    {
        stop();
        rtde_ctrl_.stopRobot();
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

    double stepTime() const
    {
        return step_time_.count();
    }

    void moveJ(const ur_control_msgs::MoveJoint& m)
    {
        if (state_ == SERVOING) {
            ROS_WARN("Discarding MoveJ command - currently servoing");
            return;
        }

        enqueueCommand([this, m]() {
            double speed = (m.speed != 0.0) ? m.speed : 1.05;
            double acceleration = (m.acceleration != 0.0) ? m.acceleration : 1.4;
            state_ = MOVING;
            rtde_ctrl_.moveJ(m.position, speed, acceleration); // blocks until robot is done moving
            state_ = IDLE;
        });
    }

    void moveL(const ur_control_msgs::MovePose& m)
    {
        if (state_ == SERVOING) {
            ROS_WARN("Discarding MoveL command - currently servoing");
            return;
        }

        enqueueCommand([this, m]() {
            double speed = (m.speed != 0.0) ? m.speed : 0.25;
            double acceleration = (m.acceleration != 0.0) ? m.acceleration : 1.2;
            state_ = MOVING;
            rtde_ctrl_.moveL(convert(m.pose), speed, acceleration); // blocks until robot is done moving
            state_ = IDLE;
        });
    }

    void servoJ(const ur_control_msgs::ServoJoint& m)
    {
        if (state_ == MOVING) {
            ROS_WARN("Discarding servoJ command - currently moving");
            return;
        }

        enqueueCommand([this, m]() {
            double lookahead_time = (m.lookahead_time != 0.0) ? m.lookahead_time : 0.1;
            double gain = (m.gain != 0.0) ? m.gain : 300;

            if (state_ != SERVOING)
                ROS_INFO("SERVOING STARTING");

            state_ = SERVOING; // will be cleared if we don't keep receiving servo commands
            auto t = std::chrono::steady_clock::now() + step_time_;
            rtde_ctrl_.servoJ(m.position, 0.0, 0.0, step_time_.count(), lookahead_time, gain); // not blocking
            std::this_thread::sleep_until(t);
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

    void processCommands()
    {
        // Process servo commands from the queue
        while (!cmd_proc_stopped_) {
            std::unique_lock<std::mutex> lock(cmd_queue_mtx_);

            // Wait for queue to be non-empty or stop, but time out after given
            bool timeout = !cmd_cv_.wait_for(lock,
                                             servo_timeout_duration_,
                                             [this]() { return !cmd_queue_.empty() || cmd_proc_stopped_; });

            if (cmd_proc_stopped_)
                break;

            if (timeout) {
                // Clear SERVOING state after timeout (ie. no new servo messages
                // were received after an interval of servo_timeout_duration_)
                if (state_ == SERVOING) {
                    rtde_ctrl_.servoStop();
                    state_ = IDLE;
                    ROS_INFO("SERVOING STOPPED");
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
};

int main(int argc, char* argv[])
{
    using namespace std::string_literals;

    ros::init(argc, argv, "ur_control");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    auto publish_tcp_pose = nh_priv.param("publish_tcp_pose", false);
    auto prefix = nh_priv.param("prefix", ""s);
    auto hostname = nh_priv.param("hostname", ""s);
    auto port = nh_priv.param("port", 30004);

    if (hostname.empty())
        throw std::runtime_error("The ~hostname parameter is not set");

    Receiver receiver("base", prefix, hostname, port);
    Controller controller(hostname, port);

    auto pub_joint_state = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    auto pub_tcp_pose = (publish_tcp_pose) ? nh.advertise<geometry_msgs::PoseStamped>("tcp_pose_current", 1) : ros::Publisher();

    std::list<ros::Subscriber> subscribers{
        nh.subscribe("move_j", 10, &Controller::moveJ, &controller, ros::TransportHints().tcpNoDelay()),
        nh.subscribe("move_l", 10, &Controller::moveL, &controller, ros::TransportHints().tcpNoDelay()),
        nh.subscribe("servo_j", 10, &Controller::servoJ, &controller, ros::TransportHints().tcpNoDelay()),
    };

    // Schedule timer to publish robot state
    auto timer = nh.createWallTimer(ros::WallDuration(controller.stepTime()),
                                    [&](const auto&) {
                                        pub_joint_state.publish(receiver.getJointStateMsg());

                                        if (publish_tcp_pose)
                                            pub_tcp_pose.publish(receiver.getActualTCPPoseStampedMsg());
                                    });

    controller.start();
    ros::spin();
    controller.stop();

    return EXIT_SUCCESS;
}
