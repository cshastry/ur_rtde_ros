#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include "rclcpp/rclcpp.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"

#include <Eigen/Geometry>

#include <boost/circular_buffer.hpp>

#include <condition_variable>
#include <mutex>
#include <thread>

#define DEFAULT_CONTROL_RATE_HZ 125.0
#define DEFAULT_UPDATE_RATE_HZ 125.0
#define DEFAULT_SERVO_J_LOOKAHEAD_TIME 0.1
#define DEFAULT_SERVO_J_GAIN 300.0

using namespace std::chrono_literals;

using secondsf = std::chrono::duration<double>;
using millisecondsf = std::chrono::duration<double, std::milli>;

geometry_msgs::msg::Pose convert_pose(const std::vector<double>& pose)
{
    assert(pose.size() == 6);
    Eigen::Vector3d r(pose[3], pose[4], pose[5]); // rotation vector
    Eigen::Quaterniond q(Eigen::AngleAxisd(r.norm(), r.normalized()));
    geometry_msgs::msg::Pose m(rosidl_runtime_cpp::MessageInitialization::SKIP);
    m.position.x = pose[0];
    m.position.y = pose[1];
    m.position.z = pose[2];
    m.orientation.w = q.w();
    m.orientation.x = q.x();
    m.orientation.y = q.y();
    m.orientation.z = q.z();
    return m;
}

std::vector<double> convert_pose(const geometry_msgs::msg::Pose& m)
{
    Eigen::Quaterniond q(m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z);
    Eigen::AngleAxisd aa(q);
    Eigen::Vector3d r = aa.axis() * aa.angle();
    return {m.position.x, m.position.y, m.position.z, r[0], r[1], r[2]};
}

geometry_msgs::msg::Twist convert_twist(const std::vector<double>& twist)
{
    assert(twist.size() == 6);
    geometry_msgs::msg::Twist m;
    m.linear.x = twist[0];
    m.linear.y = twist[1];
    m.linear.z = twist[2];
    m.angular.x = twist[3];
    m.angular.y = twist[4];
    m.angular.z = twist[5];
    return m;
}

std::vector<double> convert_twist(const geometry_msgs::msg::Twist& m)
{
    return {m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z};
}

class URReceiverNode : public rclcpp::Node
{
public:
    explicit URReceiverNode()
        : rclcpp::Node("ur_receiver")
    {
        bool publish_tcp_pose = declare_parameter<bool>("publish_tcp_pose", true);
        bool publish_tcp_twist = declare_parameter<bool>("publish_tcp_twist", true);
        auto hostname = declare_parameter<std::string>("hostname", "");
        auto prefix = declare_parameter<std::string>("prefix", "");

        std::vector<std::string> vars = {"actual_q", "actual_qd", "actual_current"};

        if (publish_tcp_pose)
            vars.push_back("actual_TCP_pose");

        if (publish_tcp_twist)
            vars.push_back("actual_TCP_speed");

        rtde_recv_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(hostname, -1, vars);

        base_frame_ = prefix + "base_link";
        tool_frame_ = prefix + "ee_link";

        // Joint names corresponding to the names in the ur_description and
        // ur_e_description ROS packages
        joint_names_ = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        };

        for (auto& n : joint_names_)
            n = prefix + n;

        pub_joints_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());

        if (publish_tcp_pose)
            pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("tcp_pose_current", rclcpp::SensorDataQoS());

        if (publish_tcp_twist)
            pub_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>("tcp_twist_current", rclcpp::SensorDataQoS());

        timer_ = create_wall_timer(8ms, [this]() { return publish_state(); }); // FIXME hard-coded publish state at 125 Hz
    }

private:
    void publish_state()
    {
        auto now = this->now();

        sensor_msgs::msg::JointState m_joint_state(rosidl_runtime_cpp::MessageInitialization::SKIP);
        m_joint_state.header.stamp = now;
        m_joint_state.name = joint_names_;
        m_joint_state.position = rtde_recv_->getActualQ();
        m_joint_state.velocity = rtde_recv_->getActualQd();
        m_joint_state.effort = rtde_recv_->getActualCurrent();
        pub_joints_->publish(m_joint_state);

        if (pub_pose_) {
            geometry_msgs::msg::PoseStamped m_pose(rosidl_runtime_cpp::MessageInitialization::SKIP);
            m_pose.header.stamp = now;
            m_pose.header.frame_id = base_frame_;
            m_pose.pose = convert_pose(rtde_recv_->getActualTCPPose());
            pub_pose_->publish(m_pose);
        }

        if (pub_twist_) {
            geometry_msgs::msg::TwistStamped m_twist(rosidl_runtime_cpp::MessageInitialization::SKIP);
            m_twist.header.stamp = now;
            m_twist.header.frame_id = tool_frame_;
            m_twist.twist = convert_twist(rtde_recv_->getActualTCPSpeed());
            pub_twist_->publish(m_twist);
        }
    }

private:
    std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_recv_;
    std::string base_frame_;
    std::string tool_frame_;
    std::vector<std::string> joint_names_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// UR control node.
//
// Maintains its own thread for processing robot commands because
//  1) the MoveX commands are blocking and we don't want them to block the ROS
//     event loop; and
//  2) the ServoX commands needs to be sent periodically, thus also requiring
//     control of the timing between calls.
class URControllerNode : public rclcpp::Node
{
    enum class State
    {
        READY,
        FREEDRIVE,
        MOVING,
        SERVOING,
        SPEEDING,
    };

public:
    explicit URControllerNode()
        : rclcpp::Node("ur_controller")
        , state_(State::READY)
        , cmd_queue_(10) // limited queue size
        , cmd_loop_stopped_(true)
    {
        auto hostname = declare_parameter<std::string>("hostname", "");
        auto prefix = declare_parameter<std::string>("prefix", "");
        auto servo_rate_hz = declare_parameter<double>("servo_rate_hz", 0);
        servo_j_lookahead_time_ = declare_parameter<double>("servo_j_lookahead_time", DEFAULT_SERVO_J_LOOKAHEAD_TIME);
        servo_j_gain_ = declare_parameter<double>("servo_j_gain", DEFAULT_SERVO_J_GAIN);

        base_frame_ = prefix + "base_link";
        rtde_ctrl_ = std::make_unique<ur_rtde::RTDEControlInterface>(hostname);

        if (servo_rate_hz == 0) {
            if (auto ur_step_time = rtde_ctrl_->getStepTime(); ur_step_time != 0)
                servo_rate_hz = 1.0 / ur_step_time;
            else
                servo_rate_hz = DEFAULT_CONTROL_RATE_HZ;

            set_parameter(rclcpp::Parameter("servo_rate_hz", servo_rate_hz));
        }

        rate_ = std::make_unique<rclcpp::WallRate>(servo_rate_hz);

        subscribers_ = {
            create_subscription<sensor_msgs::msg::JointState>(
                "servo_joint",
                rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::JointState::UniquePtr m){ servo_joint(*m); }
            ),
            create_subscription<sensor_msgs::msg::JointState>(
                "move_joint",
                rclcpp::ServicesQoS(),
                [this](const sensor_msgs::msg::JointState::UniquePtr m){ move_joint(*m); }
            ),
            create_subscription<sensor_msgs::msg::JointState>(
                "speed_joint",
                rclcpp::ServicesQoS(),
                [this](const sensor_msgs::msg::JointState::UniquePtr m){ speed_joint(*m); }
            ),
            create_subscription<geometry_msgs::msg::PoseStamped>(
                "move_linear",
                rclcpp::ServicesQoS(),
                [this](const geometry_msgs::msg::PoseStamped::UniquePtr m){ move_linear(*m); }
            ),
            create_subscription<std_msgs::msg::Bool>(
                "teach_mode_enable",
                rclcpp::ServicesQoS(),
                [this](const std_msgs::msg::Bool::UniquePtr m){ set_teach_mode_enabled(*m); }
            ),
        };

        RCLCPP_INFO(get_logger(), "Servo rate: %.2f Hz", 1.0/secondsf(rate_->period()).count());

        thread_ = std::thread([this] { process_commands(); });
    }

    ~URControllerNode()
    {
        cmd_loop_stopped_ = true;
        cmd_cv_.notify_all();
        thread_.join();

        if (rtde_ctrl_)
            rtde_ctrl_->stopScript();
    }

    // Servo (i.e., small steps, no interpolation) to a target in joint space
    void servo_joint(const sensor_msgs::msg::JointState& m)
    {
        if (state_ != State::READY) {
            RCLCPP_WARN(get_logger(), "Discarding 'servo_joint' command - not ready!");
            return;
        }

        enqueue_command([this, q = m.position]() {
            if (state_ != State::SERVOING) {
                rate_->reset();
                state_ = State::SERVOING; // cleared if we don't keep receiving servo commands for some time
            }

            // ur_rtde servoJ is non-blocking
            if (!rtde_ctrl_->servoJ(q,                                 // desired joint angles
                                    0,                                 // speed (not used)
                                    0,                                 // acceleration (not used)
                                    secondsf(rate_->period()).count(), // control time
                                    servo_j_lookahead_time_,           // look-ahead time in [0.03,0.2]
                                    servo_j_gain_))                    // P gain in [100,2000]
                RCLCPP_WARN(get_logger(), "ServoJ command failed");

            if (!rate_->sleep())
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "ServoJ cycle time overstepped");
        });
    }

    // Move (with interpolation) to a target in joint space (MoveJ)
    void move_joint(const sensor_msgs::msg::JointState& m)
    {
        if (state_ != State::READY) {
            RCLCPP_WARN(get_logger(), "Discarding 'move_joint' command - not ready!");
            return;
        }

        enqueue_command([this, q = m.position]() {
            RCLCPP_INFO(get_logger(), "Moving!");
            state_ = State::MOVING;

            // blocks until robot is done moving
            if (!rtde_ctrl_->moveJ(q, 1.05, 1.4, false))
                RCLCPP_WARN(get_logger(), "MoveJ command failed");

            state_ = State::READY;
        });
    }

   // Move (with linear interpolation) to target in tool space (MoveL)
    void move_linear(const geometry_msgs::msg::PoseStamped& m)
    {
        if (state_ != State::READY) {
            RCLCPP_WARN(get_logger(), "Discarding 'move_linear' command - not ready!");
            return;
        }

        if (m.header.frame_id != base_frame_) {
            RCLCPP_WARN_STREAM(get_logger(),
                               "Discarding MoveL command with target frame "
                                << "'" << m.header.frame_id << "'"
                                << "(expected "
                                << "'" << base_frame_ << "')");
            return;
        }

        enqueue_command([this, pose = convert_pose(m.pose)]() {
            state_ = State::MOVING;

            // blocks until robot is done moving
            if (!rtde_ctrl_->moveL(pose, 0.25, 1.2, false))
                RCLCPP_WARN(get_logger(), "MoveL command failed");

            state_ = State::READY;
        });
    }

    // Move at constant target velocity in joint space (linear acceleration profile)
    void speed_joint(const sensor_msgs::msg::JointState& m)
    {
        if (state_ != State::READY) {
            RCLCPP_WARN(get_logger(), "Discarding 'speed_joint' command - not ready!");
            return;
        }

        enqueue_command([this, qd = m.velocity]() {
            if (state_ != State::SPEEDING) {
                state_ = State::SPEEDING; // cleared if we don't keep receiving speed commands for some time
            }

            if (!rtde_ctrl_->speedJ(qd))
                RCLCPP_WARN(get_logger(), "SpeedJ command failed");
        });
    }

    // Move at constant target velocity in tool space (linear acceleration profile)
    void speed_linear(const geometry_msgs::msg::TwistStamped& m)
    {
        if (state_ != State::READY) {
            RCLCPP_WARN(get_logger(), "Discarding 'speed_joint' command - not ready!");
            return;
        }

        enqueue_command([this, xd = convert_twist(m.twist)]() {
            if (state_ != State::SPEEDING) {
                state_ = State::SPEEDING; // cleared if we don't keep receiving speed commands for some time
            }

            if (!rtde_ctrl_->speedL(xd))
                RCLCPP_WARN(get_logger(), "SpeedL command failed");
        });
    }

    void set_teach_mode_enabled(const std_msgs::msg::Bool& msg)
    {
        if (msg.data && state_ == State::READY) {
            rtde_ctrl_->teachMode();
            state_ = State::FREEDRIVE;
        } else if (!msg.data && state_ == State::FREEDRIVE) {
            rtde_ctrl_->endTeachMode();
            state_ = State::READY;
        }
    }

private:
    template<typename Callable>
    void enqueue_command(Callable&& f)
    {
        {
            std::lock_guard<std::mutex> lock(cmd_queue_mtx_);
            cmd_queue_.push_back(std::move(f));
        }

        cmd_cv_.notify_one();
    }

    // Process servo commands from the queue in a loop
    void process_commands()
    {
        cmd_loop_stopped_ = false;

        while (!cmd_loop_stopped_) {
            std::unique_lock<std::mutex> lock(cmd_queue_mtx_);

            // Wait for queue to be non-empty or stop, but time out after the
            // given period of time
            bool timeout = !cmd_cv_.wait_for(lock,
                                             10 * rate_->period(), // TODO Ten times the servo loop period?
                                             [this] { return !cmd_queue_.empty() || cmd_loop_stopped_; });

            if (cmd_loop_stopped_)
                break;

            if (timeout) {
                // Clear SERVOING state after timeout (ie. no new servo
                // messages were received after a period of time)
                if (state_ == State::SERVOING) {
                    rtde_ctrl_->servoStop();
                    cmd_queue_.clear();
                    state_ = State::READY;
                }

                // Clear SPEEDING state after timeout
                if (state_ == State::SPEEDING) {
                    rtde_ctrl_->speedStop();
                    cmd_queue_.clear();
                    state_ = State::READY;
                }
            } else {
                if (!cmd_queue_.empty()) {
                    auto cmd = std::move(cmd_queue_.front());
                    cmd_queue_.pop_front();
                    lock.unlock(); // unlock before executing command
                    cmd();
                }
            }
        }

        if (state_ == State::SERVOING)
            rtde_ctrl_->servoStop();
        else if (state_ == State::SPEEDING)
            rtde_ctrl_->speedStop();

        state_ = State::READY;
    }

private:
    std::unique_ptr<ur_rtde::RTDEControlInterface> rtde_ctrl_;
    std::string base_frame_;
    std::atomic<State> state_;
    boost::circular_buffer<std::function<void()>> cmd_queue_;
    std::condition_variable cmd_cv_;
    std::mutex cmd_queue_mtx_;
    rclcpp::WallRate::UniquePtr rate_;
    std::list<rclcpp::SubscriptionBase::SharedPtr> subscribers_;
    std::thread thread_;
    std::atomic<bool> cmd_loop_stopped_;
    double servo_j_lookahead_time_;
    double servo_j_gain_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto receiver_node = std::make_shared<URReceiverNode>();
    auto control_node = std::make_shared<URControllerNode>();

    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(receiver_node);
    executor.add_node(control_node);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
