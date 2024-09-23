#include <rc_command_lcmt.hpp>
#include <aliengo_leg_cmd_lcmt.hpp>
#include <aliengo_leg_data_lcmt.hpp>
#include <aliengo_body_data_lcmt.hpp>

#include <unitree_legged_sdk.h>
#include <unitree_joystick.h>
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

using namespace UNITREE_LEGGED_SDK;

// low cmd
constexpr uint16_t TARGET_PORT = 8007;
constexpr uint16_t LOCAL_PORT = 8082;
constexpr char TARGET_IP[] = "192.168.123.10"; 

const int LOW_CMD_LENGTH = 610;
const int LOW_STATE_LENGTH = 771;

class AliengoControl{
public:
    AliengoControl(uint8_t level):
    safe(LeggedType::Aliengo),
    udp(LOCAL_PORT, TARGET_IP, TARGET_PORT, LOW_CMD_LENGTH, LOW_STATE_LENGTH){
        udp.InitCmdData(cmd);
        cmd.levelFlag = LOWLEVEL;
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    void init();
    void handleActions(const lcm::ReceiveBuffer *rbuf, const std::string &channel, const aliengo_leg_cmd_lcmt *msg);

    Safety safe;
    UDP udp;
    
    LowCmd cmd = {0};
    LowState state = {0};

    float qInit[3] = {0};
    float qDes[3] = {0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};
    float Kd[3] = {0};
    
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002; // 0.001~0.01

    lcm::LCM aliengoLCM;    // subscribe the cmd and publish the data
    std::thread aliengoLcmThread;

    aliengo_leg_data_lcmt jointState{};    // the leg state received from the robot
    aliengo_body_data_lcmt bodyState{};    // the body state received from the robot
    xRockerBtnDataStruct gamepadData{};          // the gamepad data from unitree
    rc_command_lcmt rcCommand{};                 // the command sent to the robot
    aliengo_leg_cmd_lcmt jointCmd{};       // the cmd sent to the robot
    
    uint8_t mode = 0;
    bool firstRun = false;

};


void AliengoControl::init(){
    aliengoLCM.subscribe("command_for_robot", &AliengoControl::handleActions, this);
    std::cout << "subscribed the cmd from the LCM." << std::endl;

    aliengoLcmThread = std::thread([this](){
        while(true){
            aliengoLCM.handle();
        }
    });
    std::cout << "LCM is initialized." << std::endl;

    firstRun = true;

    // set the initial pose

    for(int i = 0; i < 12; i++){
        jointCmd.qd_des[i] = 0;
        jointCmd.tau_ff[i] = 0;
        jointCmd.kp[i] = 20.;
        jointCmd.kd[i] = 0.5;
    }

    jointCmd.q_des[0] = -0.3;
    jointCmd.q_des[1] = 1.2;
    jointCmd.q_des[2] = -2.721;
    jointCmd.q_des[3] = 0.3;
    jointCmd.q_des[4] = 1.2;
    jointCmd.q_des[5] = -2.721;
    jointCmd.q_des[6] = -0.3;
    jointCmd.q_des[7] = 1.2;
    jointCmd.q_des[8] = -2.721;
    jointCmd.q_des[9] = 0.3;
    jointCmd.q_des[10] = 1.2;
    jointCmd.q_des[11] = -2.721;

}

void AliengoControl::handleActions(const lcm::ReceiveBuffer *rbuf, const std::string &channel, const aliengo_leg_cmd_lcmt *msg){
    // Handle the actions from the LCM
    (void) rbuf;
    (void) channel;

    jointCmd = *msg;
}

void AliengoControl::UDPRecv(){
    udp.Recv();
}

void AliengoControl::UDPSend(){
    udp.Send();
}

void AliengoControl::RobotControl(){
    motiontime++;
    udp.GetRecv(state); // Get the state of the robot
    memcpy(&gamepadData, &state.wirelessRemote[0], 40); // Get the gamepad data of the robot

    rcCommand.left_stick[0] = gamepadData.lx;
    rcCommand.left_stick[1] = gamepadData.ly;
    rcCommand.right_stick[0] = gamepadData.rx;
    rcCommand.right_stick[1] = gamepadData.ry;
    rcCommand.R1 = gamepadData.btn.components.R1;
    rcCommand.L1 = gamepadData.btn.components.L1;
    rcCommand.R2 = gamepadData.btn.components.R2;
    rcCommand.L2 = gamepadData.btn.components.L2;

    if(gamepadData.btn.components.A > 0){
        mode = 0;
    } else if(gamepadData.btn.components.B > 0){
        mode = 1;
    }else if(gamepadData.btn.components.X > 0){
        mode = 2;
    }else if(gamepadData.btn.components.Y > 0){
        mode = 3;
    }else if(gamepadData.btn.components.up > 0){
        mode = 4;
    }else if(gamepadData.btn.components.right > 0){
        mode = 5;
    }else if(gamepadData.btn.components.down > 0){
        mode = 6;
    }else if(gamepadData.btn.components.left > 0){
        mode = 7;
    }
    rcCommand.mode = mode;

    /***************** get state from aliengo *****************/
    // joint state
    for(int i = 0; i < 12; i++){
        jointState.q[i] = state.motorState[i].q;
        jointState.qd[i] = state.motorState[i].dq;
        jointState.tau_est[i] = state.motorState[i].tauEst;
    }
    // body state
    for(int i = 0; i < 4; i++){
        bodyState.quat[i] = state.imu.quaternion[i];
        bodyState.contact_estimate[i] = state.footForce[i];
        if(i < 4){
            bodyState.rpy[i] = state.imu.rpy[i];
            bodyState.aBody[i] = state.imu.accelerometer[i];
            bodyState.omegaBody[i] = state.imu.gyroscope[i];
        }
    }
    // print state and gamepad data
    if(motiontime % 100 == 0){
        std::cout << "mode: " << mode << std::endl;
        std::cout << "joint state: " << std::endl;
        for(int i = 0; i < 12; i++){
            std::cout << jointState.q[i] << " ";
        }
        std::cout << std::endl;
        std::cout << "gamepad data: " << std::endl;
        std::cout << "A: " << gamepadData.btn.components.A << " ";
        std::cout << "B: " << gamepadData.btn.components.B << " ";
        std::cout << "X: " << gamepadData.btn.components.X << " ";
        std::cout << "Y: " << gamepadData.btn.components.Y << " ";
        std::cout << "up: " << gamepadData.btn.components.up << " ";
        std::cout << "right: " << gamepadData.btn.components.right << " ";
        std::cout << "down: " << gamepadData.btn.components.down << " ";
        std::cout << "left: " << gamepadData.btn.components.left << " ";
        std::cout << std::endl;
    }
    // publish the state to the LCM
    aliengoLCM.publish("body_state_data", &bodyState);
    aliengoLCM.publish("leg_state_data", &jointState);
    aliengoLCM.publish("rc_command", &rcCommand);

    /***************** send cmd to aliengo *****************/
    /*for(int i = 0; i < 12; i++){
        cmd.motorCmd[i].q = jointCmd.q_des[i];
        cmd.motorCmd[i].dq = jointCmd.qd_des[i];
        cmd.motorCmd[i].Kp = jointCmd.kp[i];
        cmd.motorCmd[i].Kd = jointCmd.kd[i];
        cmd.motorCmd[i].tau = jointCmd.tau_ff[i];
    }

    safe.PositionLimit(cmd);
    safe.PowerProtect(cmd, state, 9);
    udp.SetSend(cmd);*/
}

int main(void){
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    AliengoControl aliengo_control(LOWLEVEL);
    aliengo_control.init();
    InitEnvironment();
    LoopFunc loop_control("control_loop", aliengo_control.dt, boost::bind(&AliengoControl::RobotControl, &aliengo_control));
    loop_control.start();
    LoopFunc loop_udp_recv("udp_recv_loop", aliengo_control.dt, 3, boost::bind(&AliengoControl::UDPRecv, &aliengo_control));
    loop_udp_recv.start();
    LoopFunc loop_udp_send("udp_send_loop", aliengo_control.dt, 3, boost::bind(&AliengoControl::UDPSend, &aliengo_control));
    loop_udp_send.start();

    while(1){
        sleep(10);
    }

    return 0;
}
