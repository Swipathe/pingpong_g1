/*
This is the transition layer for Deployment on Unitree series robot

The main logic for this code is that it serves as the transition layer between the policy(high-level) and the robots(low-level).

                                                Policy (High Level)
                                                       |
                                                Transition Layer (HERE)
                                                       |
                                                Unitree Robot (Low Level)

The code is based on the prevailing LCM to build communication between different parts and support easy transfer between any unitree robots (For example, G1->H1)

You may define the robot-specific params in unitree_sdk2/assets where you can create a new file and define the params as unitree_g1_29dof.hpp

The detailed implementation of communication is:

                                                Policy (High Level)
                                                   LCM||LCM
                                                Transition Layer (HERE)
                                   (Wrapped by LCM)DDS||DDS
                                                Unitree Robot (Low Level)

The left side represents the direction from up to down and the right side is the opposite.
*/


// Standard Content
#include <cmath>
#include <memory>
#include <algorithm>
#include <array>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include <lcm/lcm-cpp.hpp>

// Unitree
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include "unitree/common/thread/thread.hpp"

// LCM
#include "pd_tau_targets_lcmt.hpp"
#include "state_estimator_lcmt.hpp"
#include "body_control_data_lcmt.hpp"
#include "rc_command_lcmt.hpp"

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_STATE_TOPIC = "rt/lowstate";
#define TOPIC_SPORT_STATE "rt/odommodestate"

#include "assets/remote_controller.hpp" 

/*You may include different robot configs for different usages; All you need to do is to define the relevant variables in assets/[YOUR UNITREE ROBOT].hpp*/
#include "assets/unitree_g1_29dof.hpp"

namespace {

bool force_ai_sport_off() {
    unitree::robot::go2::RobotStateClient client;
    client.SetTimeout(3.0f);
    client.Init();

    int32_t returned_status = -1;
    int32_t ret = client.ServiceSwitch("ai_sport", 0, returned_status);
    std::cout << "Safety precheck: ServiceSwitch name=ai_sport switch=0 ret="
              << ret << " returned_status=" << returned_status << std::endl;
    if (ret != 0) {
        std::cout << "Safety precheck failed: cannot switch ai_sport off." << std::endl;
        return false;
    }

    std::vector<unitree::robot::go2::ServiceState> services;
    ret = client.ServiceList(services);
    std::cout << "Safety precheck: ServiceList ret=" << ret << std::endl;
    if (ret != 0) {
        std::cout << "Safety precheck failed: cannot verify ai_sport status." << std::endl;
        return false;
    }

    for (const auto& service : services) {
        if (service.name == "ai_sport") {
            std::cout << "Safety precheck: service name=ai_sport status="
                      << service.status << " protect=" << service.protect << std::endl;
            if (service.status != 0) {
                std::cout << "Safety precheck failed: ai_sport is still enabled." << std::endl;
                return false;
            }
            return true;
        }
    }

    std::cout << "Safety precheck failed: ai_sport service not found." << std::endl;
    return false;
}

}  // namespace

/*---------------------------Here is the main body of the controller-----------------------------*/
class RobotController {
    private:
        double time_;
        double control_dt_;  // 0.002s = 500Hz
        double duration_;    // time to move from current pose to default pose before policy handoff
        PRorAB mode_;        // mode for control ankle
        uint8_t mode_machine_; 

        /*Communication between control interface and low level humanoid robots*/
        unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher_;
        unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber_; //Unitree state subscriber and publisher
        unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> odometer_subscriber_;

        /*Communication between control interface and high level policy*/
        lcm::LCM _simpleLCM;
        state_estimator_lcmt body_state_simple = {0};
        body_control_data_lcmt joint_state_simple = {0};
        pd_tau_targets_lcmt joint_command_simple = {0};
        rc_command_lcmt rc_command = {0};

        /*Multi-threads*/
        unitree::common::ThreadPtr highstateWriterThreadPtr, lowcmdWriterThreadPtr, highcmdReceiverThreadPtr;

        /*Indicators*/
        bool _firstRun;
        bool _firstCommandReceived;
        bool _firstLowCmdReceived;
        bool _firstHighCmdReceived;
        bool _firstOdometerMsgReceived;
        bool damping_mode_;
        int damping_log_count_;
        bool last_l2b_pressed_;
        bool last_l2y_pressed_;
        bool startup_pose_captured_;

        /*Data buffer*/
        unitree_hg::msg::dds_::LowState_ low_state{};
        unitree_hg::msg::dds_::LowCmd_ low_cmd{};
        unitree_go::msg::dds_::SportModeState_ odometer_state{};
        xRockerBtnDataStruct remote_key_data;
        std::array<float, NUM_MOTOR> startup_joint_position_{};

        /*Mutexes for shared data*/
        std::mutex low_state_mutex_;
        std::mutex joint_command_mutex_;
        std::mutex odometer_mutex_;
        


    public:
        RobotController(std::string networkInterface): 
            time_(0.0),
            control_dt_(0.002), // 500Hz
            duration_(5.0),
            mode_(PR), // ankle control mode
            mode_machine_(0),
            damping_mode_(false),
            damping_log_count_(0),
            last_l2b_pressed_(false),
            last_l2y_pressed_(false),
            startup_pose_captured_(false)
        {
            // Init network connection
            unitree::robot::ChannelFactory::Instance()->Init(0, networkInterface);
            if (!force_ai_sport_off()) {
                throw std::runtime_error("Refusing to start transition layer until ai_sport is off.");
            }

            set_default_state();
    
            /*-------Create Communication between transition layer and the low-level humanoid robots------*/
            // create publisher (transition layer -> robot)
            lowcmd_publisher_.reset(
                new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(HG_CMD_TOPIC));
            lowcmd_publisher_->InitChannel();
            // create writer (which uses publisher) (transition layer -> robot)
            lowcmdWriterThreadPtr = unitree::common::CreateRecurrentThreadEx("dds_write_thread", UT_CPU_ID_NONE, control_dt_*1e6, &RobotController::lowcmdWriter, this);

            // create subscriber (robot -> transition layer)
            lowstate_subscriber_.reset(
                new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(
                    HG_STATE_TOPIC));
            lowstate_subscriber_->InitChannel(
                std::bind(&RobotController::lowstateHandler, this, std::placeholders::_1), 1);

            odometer_subscriber_.reset(
                new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(
                    TOPIC_SPORT_STATE));
            odometer_subscriber_->InitChannel(
                std::bind(&RobotController::OdometerHandler, this, std::placeholders::_1), 1);

            /*--------Create Comminication between transition layer and the high-level policy-------*/
            // create lcm subscriber (policy action -> transition layer); Receiver receives high-level signals and hands over to handler for processing.
            _simpleLCM.subscribe("pd_plustau_targets", &RobotController::highcmdHandler, this);
            highcmdReceiverThreadPtr = unitree::common::CreateRecurrentThreadEx("lcm_recv_thread", UT_CPU_ID_NONE, control_dt_*1e6, &RobotController::highcmdReceiver, this);

            // lcm send thread (transition layer -> policy)
            highstateWriterThreadPtr = unitree::common::CreateRecurrentThreadEx("lcm_send_thread", UT_CPU_ID_NONE, control_dt_*1e6, &RobotController::highstateWriter, this);

            
            _firstRun = true;
            _firstCommandReceived = false;
            _firstLowCmdReceived = false; 
            _firstHighCmdReceived = false;
            _firstOdometerMsgReceived = false;
        }

        /*Initialization*/
        void set_default_state(){
            for(int i=0; i<NUM_MOTOR; i++){
                joint_command_simple.q_des[i] = default_joint_position[i];
                joint_command_simple.qd_des[i] = 0;
                joint_command_simple.tau_ff[i] = 0;
                joint_command_simple.kp[i] = Kp[i];
                joint_command_simple.kd[i] = Kd[i];

            }
            std::cout << "Default Joint Position Set!" << std::endl; 
        }

        void hold_current_joint_positions(){
            // Read directly from low_state to avoid using uninitialized joint_state_simple
            std::lock_guard<std::mutex> state_lk(low_state_mutex_);
            std::lock_guard<std::mutex> command_lk(joint_command_mutex_);
            for(int i=0; i<NUM_MOTOR; i++){
                joint_command_simple.q_des[i] = low_state.motor_state()[i].q();
                joint_command_simple.qd_des[i] = 0;
                joint_command_simple.tau_ff[i] = 0;
                joint_command_simple.kp[i] = Kp[i];
                joint_command_simple.kd[i] = Kd[i];
            }
        }

        void capture_startup_joint_positions(){
            std::lock_guard<std::mutex> lk(low_state_mutex_);
            for(int i=0; i<NUM_MOTOR; i++){
                startup_joint_position_[i] = low_state.motor_state()[i].q();
            }
            startup_pose_captured_ = true;
        }

        float startup_interpolation_ratio() const {
            if(duration_ <= 0.0)
                return 1.0f;
            return static_cast<float>(std::clamp(time_ / duration_, 0.0, 1.0));
        }

        static float interpolate_startup_joint(float start, float target, float ratio) {
            ratio = std::clamp(ratio, 0.0f, 1.0f);
            return (target - start) * ratio + start;
        }

        /*-----------Communication with the high-level layer--------------*/
        // High command receive(It will hand over the control signal to handler): Policy -> Transition Layer
        void highcmdReceiver(){
            while(true){
                // handleTimeout avoids busy-spinning; 2ms matches control_dt_
                _simpleLCM.handleTimeout(2);
            }
        }

        void highcmdHandler(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const pd_tau_targets_lcmt *msg){
            (void) rbuf;
            (void) chan;
            std::lock_guard<std::mutex> lk(joint_command_mutex_);
            joint_command_simple = *msg;

            if (!_firstHighCmdReceived){
                _firstHighCmdReceived = true;
                std::cout<< "Communication built successfully between transition layer and policy!" << std::endl;
            }
        }

        //High state writer: Transition Layer -> Policy
        void highstateWriter() {
            {
                std::lock_guard<std::mutex> lk(low_state_mutex_);
                for(int i=0; i<NUM_MOTOR; i++){
                    joint_state_simple.q[i] = low_state.motor_state()[i].q();
                    joint_state_simple.qd[i] = low_state.motor_state()[i].dq();
                    joint_state_simple.tau_est[i] = low_state.motor_state()[i].tau_est();
                }

                for(int i=0; i<4; i++)
                    body_state_simple.quat[i] = low_state.imu_state().quaternion()[i];

                for(int i=0; i<3; i++){
                    body_state_simple.rpy[i] = low_state.imu_state().rpy()[i];
                    body_state_simple.aBody[i] = low_state.imu_state().accelerometer()[i];
                    body_state_simple.omegaBody[i] = low_state.imu_state().gyroscope()[i];
                }

                if(mode_machine_ != low_state.mode_machine()){
                    if(mode_machine_ == 0)
                        std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl;
                    mode_machine_ = low_state.mode_machine();
                }

                memcpy(&remote_key_data, &low_state.wireless_remote()[0], 40);
            }

            {
                std::lock_guard<std::mutex> lk(odometer_mutex_);
                if(_firstOdometerMsgReceived){
                    for(int i=0; i<3; i++){
                        body_state_simple.p[i] = odometer_state.position()[i];
                        body_state_simple.vBody[i] = odometer_state.velocity()[i];
                    }
                }
            }
            rc_command.left_stick[0] = remote_key_data.lx;
            rc_command.left_stick[1] = remote_key_data.ly;
            rc_command.right_stick[0] = remote_key_data.rx;
            rc_command.right_stick[1] = remote_key_data.ry;
            rc_command.right_lower_right_switch = remote_key_data.btn.components.R2;
            rc_command.right_upper_switch = remote_key_data.btn.components.R1;
            rc_command.left_lower_left_switch = remote_key_data.btn.components.L2;
            rc_command.left_upper_switch = remote_key_data.btn.components.L1;

            _simpleLCM.publish("state_estimator_data", &body_state_simple);
            _simpleLCM.publish("body_control_data", &joint_state_simple);
            _simpleLCM.publish("rc_command_data", &rc_command);
        }


        void lowstateHandler(const void *message) {
            /*
            The lowstateHandler is mainly responsible for the following things:
            1. Update the current proprioception state
            2. Obtain the remote controller state
            3. Update the signal across threads
            */
            std::lock_guard<std::mutex> lk(low_state_mutex_);
            low_state = *(const unitree_hg::msg::dds_::LowState_ *)message;

            if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1)) 
            {
                std::cout << "low_state CRC Error" << std::endl;
                return;
            }

            if (_firstLowCmdReceived == false)
            {
                std::cout << "Communication built successfully between transition layer and robot!" <<std::endl;
                _firstLowCmdReceived = true;
            }
        }

        void OdometerHandler(const void *message) {
            std::lock_guard<std::mutex> lk(odometer_mutex_);
            odometer_state = *(unitree_go::msg::dds_::SportModeState_ *) message;
            
            if(_firstOdometerMsgReceived == false)
            {
                std::cout << "Commnication built successfully between transition layer and odometer!" << std::endl;
                _firstOdometerMsgReceived = true;
            }
        }

        void lowcmdWriter() {

            low_cmd.mode_pr() = mode_;
            low_cmd.mode_machine() = mode_machine_;

            if (!_firstLowCmdReceived){
                return;
            }

            bool l2b_pressed = ((int) remote_key_data.btn.components.B ==1 && (int) remote_key_data.btn.components.L2 == 1);
            bool l2y_pressed = ((int) remote_key_data.btn.components.Y ==1 && (int) remote_key_data.btn.components.L2 == 1);
            bool l2b_edge = l2b_pressed && !last_l2b_pressed_;
            bool l2y_edge = l2y_pressed && !last_l2y_pressed_;
            last_l2b_pressed_ = l2b_pressed;
            last_l2y_pressed_ = l2y_pressed;

            float rpy0, rpy1;
            {
                std::lock_guard<std::mutex> lk(low_state_mutex_);
                rpy0 = low_state.imu_state().rpy()[0];
                rpy1 = low_state.imu_state().rpy()[1];
            }
            bool imu_tilt = std::abs(rpy0) > 0.8f || std::abs(rpy1) > 0.8f;
            bool damping_trigger = _firstLowCmdReceived && (l2b_edge || imu_tilt);
            if(damping_trigger && !damping_mode_){
                damping_mode_ = true;
                damping_log_count_ = 0;
                if(imu_tilt)
                    std::cout << "IMU tilt detected (rpy[0]=" << rpy0
                              << " rpy[1]=" << rpy1
                              << "), switching to Damping Mode!" << std::endl;
                else
                    std::cout << "Switched to Damping Mode!" << std::endl;
            }

            if(damping_mode_){
                for(int i=0; i<NUM_MOTOR; i++){
                    low_cmd.motor_cmd()[i].mode() = 1;
                    low_cmd.motor_cmd()[i].q() = 0;
                    low_cmd.motor_cmd()[i].dq() = 0;
                    low_cmd.motor_cmd()[i].kp() = 0;
                    low_cmd.motor_cmd()[i].kd() = 5;
                    low_cmd.motor_cmd()[i].tau() = 0;
                }

                if(l2y_edge){
                    std::cout<< "L2+Y is pressed, recover to startup interpolation." <<std::endl;
                    capture_startup_joint_positions();
                    time_ = 0.0;
                    _firstRun = true;
                    damping_mode_ = false;
                }else{
                    if(damping_log_count_ % 50 == 0){
                        std::cout<<"Press L2+Y to recover; L2+B to re-enter damping." <<std::endl;
                    }
                    damping_log_count_++;
                }
            }else if(time_ < duration_){
                if(!startup_pose_captured_){
                    capture_startup_joint_positions();
                    std::cout << "Startup interpolation captured current joint positions; moving to default pose over "
                              << duration_ << "s." << std::endl;
                }

                time_ = std::min(time_ + control_dt_, duration_);
                float ratio = startup_interpolation_ratio();
                for(int i=0; i<NUM_MOTOR; i++){
                    low_cmd.motor_cmd()[i].mode() = 1;
                    low_cmd.motor_cmd()[i].q() = interpolate_startup_joint(
                        startup_joint_position_[i],
                        default_joint_position[i],
                        ratio);
                    low_cmd.motor_cmd()[i].dq() = 0;
                    low_cmd.motor_cmd()[i].kp() = Kp[i];
                    low_cmd.motor_cmd()[i].kd() = Kd[i];
                    low_cmd.motor_cmd()[i].tau() = 0;
                }
            }else{
                if (_firstRun){
                    hold_current_joint_positions(); // reads low_state under low_state_mutex_
                    remote_key_data.btn.components.Y = 0;
                    remote_key_data.btn.components.A = 0;
                    remote_key_data.btn.components.B = 0;
                    remote_key_data.btn.components.L2 = 0;
                    _firstRun = false;
                    std::cout << "Startup interpolation complete; holding current joint positions until policy commands arrive." << std::endl;
                }

                std::lock_guard<std::mutex> lk(joint_command_mutex_);
                for(int i=0; i<NUM_MOTOR; i++){
                    low_cmd.motor_cmd()[i].mode() = 1;
                    low_cmd.motor_cmd()[i].q() = joint_command_simple.q_des[i];
                    low_cmd.motor_cmd()[i].dq() = joint_command_simple.qd_des[i];
                    low_cmd.motor_cmd()[i].kp() = joint_command_simple.kp[i];
                    low_cmd.motor_cmd()[i].kd() = joint_command_simple.kd[i];
                    low_cmd.motor_cmd()[i].tau() = joint_command_simple.tau_ff[i];
                }
            }
            
            low_cmd.crc() = Crc32Core((uint32_t *)&low_cmd, (sizeof(low_cmd)>>2)-1);
            lowcmd_publisher_->Write(low_cmd);
    }
};

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " networkInterface"<< std::endl;
    exit(-1);
  }

  std::cout << "Make sure the robot is hung up!" << std::endl
            << "The transition layer will move to default pose over 5s after lowstate is received." <<std::endl
            << "Start the policy layer only after this startup interpolation is stable." <<std::endl
            << "You may press [L2 + B] to stop the process." <<std::endl
            << "You may press [L2 + Y] to recover from stopped state." << std::endl
            << "You may double press [L1 + B] for emergency termination." << std::endl
            << "Press Enter to continue ..." <<std::endl;
  std::cin.ignore(); // Press Enter to continue

  std::string networkInterface = argv[1];
  RobotController custom(networkInterface);
  while (true) usleep(20000); // 0.02s
  return 0;
}
