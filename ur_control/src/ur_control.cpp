#include "monotonic.h"

#include <ur_control_msgs/SetServoJGain.h>
#include <ur_control_msgs/SetServoJLookaheadTime.h>
#include <ur_control_msgs/SetServoLoopRate.h>
#include <ur_control_msgs/SetTeachModeEnabled.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <ros/ros.h>

#include <Eigen/Geometry>

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#define DEFAULT_CONTROL_RATE 125.0
#define DEFAULT_SERVOJ_LOOKAHEAD_TIME 0.1
#define DEFAULT_SERVOJ_GAIN 300.0

using namespace std::chrono_literals;

using secondsf = std::chrono::duration<double>;
using millisecondsf = std::chrono::duration<double, std::milli>;

geometry_msgs::Pose convertPose(const std::vector<double>& pose)
{
    assert(pose.size() == 6);
    Eigen::Vector3d r(pose[3], pose[4], pose[5]); // rotation vector
    Eigen::Quaterniond q(Eigen::AngleAxisd(r.norm(), r.normalized()));
    geometry_msgs::Pose m;
    m.position.x = pose[0];
    m.position.y = pose[1];
    m.position.z = pose[2];
    m.orientation.w = q.w();
    m.orientation.x = q.x();
    m.orientation.y = q.y();
    m.orientation.z = q.z();
    return m;
}

std::vector<double> convertPose(const geometry_msgs::Pose& m)
{
    Eigen::Quaterniond q(m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z);
    Eigen::AngleAxisd aa(q);
    Eigen::Vector3d r = aa.axis() * aa.angle();
    return {m.position.x, m.position.y, m.position.z, r[0], r[1], r[2]};
}

geometry_msgs::Twist convertTwist(const std::vector<double>& twist)
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

std::vector<double> convertTwist(const geometry_msgs::Twist& m)
{
    return {m.linear.x, m.linear.y, m.linear.z,
            m.angular.x, m.angular.y, m.angular.z};
}

// Wrapper around ur_rtde::RTDEReceiveInterface to bridge with ROS message types.
class Receiver
{
public:
    explicit Receiver(const std::string& base_frame,
                      const std::string& tool_frame,
                      const std::string& prefix,
                      const std::string& hostname,
                      const std::vector<std::string>& variables)
        : rtde_recv_(hostname, -1, variables)
        , base_frame_(prefix + base_frame)
        , tool_frame_(prefix + tool_frame)
    {
        // Joint names corresponding to the names in the ur_description and
        // ur_e_description ROS packages
        static const std::vector<std::string> base_joint_names = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        };

        std::transform(base_joint_names.begin(),
                       base_joint_names.end(),
                       std::back_inserter(joint_names_),
                       [&](const auto& n) { return prefix + n; });
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
        m.pose = convertPose(rtde_recv_.getActualTCPPose());
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
    enum class State
    {
        IDLE,
        FREEDRIVE,
        MOVING,
        SERVOING,
        SPEEDING,
    };

public:
    explicit Controller(const std::string& base_frame,
                        const std::string& prefix,
                        const std::string& hostname)
        : rtde_ctrl_(hostname)
        , base_frame_(prefix + base_frame)
        , state_(State::IDLE)
        , cmd_proc_stopped_(true)
        , rate_(DEFAULT_CONTROL_RATE)
        , servoj_lookahead_time_(DEFAULT_SERVOJ_LOOKAHEAD_TIME)
        , servoj_gain_(DEFAULT_SERVOJ_GAIN)
    {
        if (ros::param::has("~servo_rate")) {
            setLoopRate(ros::param::param<double>("~servo_rate", getStepTime()));
        } else {
            auto ur_step_time = rtde_ctrl_.getStepTime();

            // It seems RTDEControlInterface::getStepTime() returns zero for
            // simulated UR robots, so we correct for that here
            if (ur_step_time == 0)
                ROS_WARN("'%s' returned step time 0, defaulting to %.2f ms (%.2f Hz)",
                         hostname.c_str(),
                         getStepTime() * 1000,
                         1.0 / getStepTime());
            else
                setLoopRate(1.0 / ur_step_time);
        }
    }

    ~Controller()
    {
        rtde_ctrl_.stopScript();
    }

    void run()
    {

        cmd_proc_stopped_ = false;
        processCommands();
    }

    void stop()
    {
        cmd_proc_stopped_ = true;
        cmd_cv_.notify_all();
    }

    void setLoopRate(double f)
    {
        rate_ = monotonic_rate(f);
        ROS_INFO("Setting control loop period to %.2f ms (%.2f Hz)", getStepTime() * 1000, 1.0 / getStepTime());
    }

    double getStepTime() const
    {
        return secondsf(rate_.expected_cycle_time()).count();
    }

    void moveJ(const sensor_msgs::JointState& m)
    {
        if (state_ != State::IDLE && state_ != State::MOVING) {
            ROS_WARN("Discarding MoveJ command - not ready");
            return;
        }

        enqueueCommand([this, q = m.position]() {
            state_ = State::MOVING;

            // blocks until robot is done moving
            if (!rtde_ctrl_.moveJ(q, 1.05, 1.4))
                ROS_WARN("MoveJ command failed");

            state_ = State::IDLE;
        });
    }

    void servoJ(const sensor_msgs::JointState& m)
    {
        if (state_ != State::IDLE && state_ != State::SERVOING) {
            ROS_WARN("Discarding servoJ command - not ready");
            return;
        }

        enqueueCommand([this, q = m.position]() {
            if (state_ != State::SERVOING) {
                rate_.reset();
                state_ = State::SERVOING; // cleared if we don't keep receiving servo commands for some time
            }

            // ur_rtde servoJ is non-blocking
            if (!rtde_ctrl_.servoJ(q,                      // desired joint angles
                                   0,                      // speed (not used)
                                   0,                      // acceleration (not used)
                                   getStepTime(),          // control time
                                   servoj_lookahead_time_, // look-ahead time in [0.03,0.2]
                                   servoj_gain_))          // P gain in [100,2000]
                ROS_WARN("ServoJ command failed");

            if (!rate_.sleep())
                ROS_WARN_THROTTLE(0.5, "ServoJ cycle time overstepped (expected: %.2f ms, actual: %.2f ms)", millisecondsf(rate_.expected_cycle_time()).count(), millisecondsf(rate_.actual_cycle_time()).count());
        });
    }

    void moveL(const geometry_msgs::PoseStamped& m)
    {
        if (state_ != State::IDLE && state_ != State::MOVING) {
            ROS_WARN("Discarding MoveL command - not ready!");
            return;
        }

        if (m.header.frame_id != base_frame_) {
            ROS_WARN_STREAM("Discarding MoveL command with target frame "
                            << "'" << m.header.frame_id << "'"
                            << "(expected "
                            << "'" << base_frame_ << "')");
            return;
        }

        enqueueCommand([this, pose = convertPose(m.pose)]() {
            state_ = State::MOVING;

            // blocks until robot is done moving
            if (!rtde_ctrl_.moveL(pose, 0.25, 1.2))
                ROS_WARN("MoveL command failed");

            state_ = State::IDLE;
        });
    }

    // Move at constant target velocity in joint space (linear acceleration profile)
    void speedJ(const sensor_msgs::JointState& m)
    {
        if (state_ != State::IDLE && state_ != State::SPEEDING) {
            ROS_WARN("Discarding speedJ command - not ready!");
            return;
        }

        enqueueCommand([this, qd = m.velocity]() {
            if (state_ != State::SPEEDING) {
                state_ = State::SPEEDING; // cleared if we don't keep receiving speed commands for some time
            }

            // ur_rtde servoJ is non-blocking
            if (!rtde_ctrl_.speedJ(qd,  // desired joint velocities
                                   0.5, // acceleration
                                   0))  // blocking time for this function call
                ROS_WARN("SpeedJ command failed");
        });
    }

    // Move at constant target velocity in joint space (linear acceleration profile)
    void speedL(const geometry_msgs::TwistStamped& m)
    {
        if (state_ != State::IDLE && state_ != State::SPEEDING) {
            ROS_WARN("Discarding speedL command - not ready!");
            return;
        }

        enqueueCommand([this, xd = convertTwist(m.twist)]() {
            if (state_ != State::SPEEDING) {
                state_ = State::SPEEDING; // cleared if we don't keep receiving speed commands for some time
            }

            // ur_rtde servoJ is non-blocking
            if (!rtde_ctrl_.speedL(xd,   // desired joint velocities
                                   0.25, // acceleration
                                   0))   // blocking time for this function call
                ROS_WARN("speedL command failed");
        });
    }

    bool setServoJLookaheadTime(ur_control_msgs::SetServoJLookaheadTime::Request& req,
                                ur_control_msgs::SetServoJLookaheadTime::Response&)
    {
        servoj_lookahead_time_ = req.lookahead_time;
        return true;
    }

    bool setServoJGain(ur_control_msgs::SetServoJGain::Request& req,
                       ur_control_msgs::SetServoJGain::Response&)
    {
        servoj_gain_ = req.gain;
        return true;
    }

    void setTeachModeEnabled(const std_msgs::Bool& msg)
    {
        if (msg.data && state_ == State::IDLE) {
            rtde_ctrl_.teachMode();
            state_ = State::FREEDRIVE;
        } else if (!msg.data && state_ == State::FREEDRIVE) {
            rtde_ctrl_.endTeachMode();
            state_ = State::IDLE;
        }
    }

private:
    template<typename Callable>
    void enqueueCommand(Callable&& f)
    {
        {
            std::lock_guard<std::mutex> lock(cmd_queue_mtx_);
            cmd_queue_.push(std::move(f));
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
                                             10 * rate_.expected_cycle_time(), // ten times the servo loop period
                                             [this] { return !cmd_queue_.empty() || cmd_proc_stopped_; });

            if (cmd_proc_stopped_)
                break;

            if (timeout) {
                // Clear SERVOING state after timeout (ie. no new servo
                // messages were received after a period of time)
                if (state_ == State::SERVOING) {
                    rtde_ctrl_.servoStop();
                    cmd_queue_ = {}; // clear queue
                    state_ = State::IDLE;
                }

                // Clear SPEEDING state after timeout
                if (state_ == State::SPEEDING) {
                    rtde_ctrl_.speedStop();
                    cmd_queue_ = {}; // clear queue
                    state_ = State::IDLE;
                }
            } else {
                auto cmd = std::move(cmd_queue_.front());
                cmd_queue_.pop();
                lock.unlock(); // unlock before the executing command
                cmd();
            }
        }

        if (state_ == State::SERVOING)
            rtde_ctrl_.servoStop();
        else if (state_ == State::SPEEDING)
            rtde_ctrl_.speedStop();

        state_ = State::IDLE;
    }

private:
    ur_rtde::RTDEControlInterface rtde_ctrl_;
    std::string base_frame_;
    std::atomic<State> state_;
    std::atomic<bool> cmd_proc_stopped_;
    std::condition_variable cmd_cv_;
    std::mutex cmd_queue_mtx_;
    std::queue<std::function<void()>> cmd_queue_;
    std::thread thread_;
    monotonic_rate rate_;
    double servoj_lookahead_time_;
    double servoj_gain_;
};

int main(int argc, char* argv[])
{
    using namespace std::string_literals;

    // Lock memory and handle page faults immediately for improved realtime performance
    //try {
    //    lock_and_prefault_mem(32 * 1024 * 1024);
    //} catch (const std::runtime_error& e) {
    //    std::cerr << "Lock/prefault memory failed: " << e.what() << std::endl;
    //}

    ros::init(argc, argv, "ur_control");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    auto publish_tcp_pose = nh_priv.param("publish_tcp_pose", false);
    auto publish_tcp_twist = nh_priv.param("publish_tcp_twist", false);
    auto prefix = nh_priv.param("prefix", ""s);
    auto hostname = nh_priv.param("hostname", ""s);

    if (hostname.empty())
        throw std::runtime_error("The ~hostname parameter is not set");

    std::vector<std::string> rtde_output_variables = {"actual_q", "actual_qd", "actual_current"};

    if (publish_tcp_pose)
        rtde_output_variables.push_back("actual_TCP_pose");

    if (publish_tcp_twist)
        rtde_output_variables.push_back("actual_TCP_speed");

    Receiver receiver("base_link", "ee_link", prefix, hostname, rtde_output_variables);
    Controller controller("base_link", prefix, hostname);

    auto pub_joint_state = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    auto pub_tcp_pose = (publish_tcp_pose) ? nh.advertise<geometry_msgs::PoseStamped>("tcp_pose_current", 1) : ros::Publisher{};
    auto pub_tcp_twist = (publish_tcp_twist) ? nh.advertise<geometry_msgs::TwistStamped>("tcp_twist_current", 1) : ros::Publisher{};

    std::list<ros::ServiceServer> service_servers{
        nh.advertiseService("set_servo_joint_lookahead_time", &Controller::setServoJLookaheadTime, &controller),
        nh.advertiseService("set_servo_joint_gain", &Controller::setServoJGain, &controller),
    };

    std::list<ros::Subscriber> subscribers{
        nh.subscribe("move_joint", 2, &Controller::moveJ, &controller, ros::TransportHints().tcp().tcpNoDelay()),
        nh.subscribe("move_tool_linear", 2, &Controller::moveL, &controller, ros::TransportHints().tcp().tcpNoDelay()),
        nh.subscribe("servo_joint", 1, &Controller::servoJ, &controller, ros::TransportHints().udp().tcp().tcpNoDelay()),
        nh.subscribe("speed_joint", 1, &Controller::speedJ, &controller, ros::TransportHints().udp().tcp().tcpNoDelay()),
        nh.subscribe("speed_linear", 1, &Controller::speedL, &controller, ros::TransportHints().udp().tcp().tcpNoDelay()),
        nh.subscribe("teach_mode_enable", 10, &Controller::setTeachModeEnabled, &controller, ros::TransportHints().tcp().tcpNoDelay()),
    };

    // Schedule timer to publish robot state
    auto timer = nh.createSteadyTimer(
        ros::WallDuration(controller.getStepTime()),
        [&](const auto&) {
            pub_joint_state.publish(receiver.getJointStateMsg());

            if (publish_tcp_pose)
                pub_tcp_pose.publish(receiver.getActualTCPPoseMsg());

            if (publish_tcp_twist)
                pub_tcp_pose.publish(receiver.getActualTCPTwistMsg());
        });

    auto thread_control = std::thread([&controller] {
        // Set realtime priority for this thread
        //try {
        //    thread_set_sched_fifo_with_priority(pthread_self(), 80);
        //} catch (const std::runtime_error& e) {
        //    ROS_WARN_STREAM("Setting realtime thread priority failed: " << e.what());
        //}

        controller.run();
    });

    ros::spin();
    controller.stop();
    thread_control.join();

    return EXIT_SUCCESS;
}
