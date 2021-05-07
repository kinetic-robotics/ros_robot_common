#include <boost/algorithm/string/case_conv.hpp>

#include <robot_toolbox/tool.h>

#include "rm_control/channel/keyboard.h"

namespace rm_control
{
KeyboardChannel::KeyboardChannel(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : ChannelInterface(node, nodeParam_), node_(node), nodeParam_(nodeParam)
{
}

bool KeyboardChannel::checkKey(std::string& key)
{
    std::array<std::string, 16> checkArray = {"W", "S", "A", "D", "SHIFT", "CTRL", "Q", "E", "R", "F", "G", "Z", "X", "C", "V", "B"};
    return std::find(checkArray.begin(), checkArray.end(), boost::to_upper_copy(key)) != checkArray.end();
}

bool KeyboardChannel::getValueByKeyName(std::string& key, const rm_rc_controller::KeyboardConstPtr& keyboard)
{
    if (key == "W") return keyboard->W;
    if (key == "S") return keyboard->S;
    if (key == "A") return keyboard->A;
    if (key == "D") return keyboard->D;
    if (key == "SHIFT") return keyboard->SHIFT;
    if (key == "CTRL") return keyboard->CTRL;
    if (key == "Q") return keyboard->Q;
    if (key == "E") return keyboard->E;
    if (key == "R") return keyboard->R;
    if (key == "F") return keyboard->F;
    if (key == "G") return keyboard->G;
    if (key == "Z") return keyboard->Z;
    if (key == "X") return keyboard->X;
    if (key == "C") return keyboard->C;
    if (key == "V") return keyboard->V;
    if (key == "B") return keyboard->B;
    return false;
}

void KeyboardChannel::keyboardCallback(const rm_rc_controller::KeyboardConstPtr& msg)
{
    bool isVxForwardPress  = getValueByKeyName(vxForwardKey_, msg);  /* X轴前进按键是否按下 */
    bool isVxBackwardPress = getValueByKeyName(vxBackwardKey_, msg); /* X轴后退按键是否按下 */
    bool isVyForwardPress  = getValueByKeyName(vyForwardKey_, msg);  /* Y轴前进按键是否按下 */
    bool isVyBackwardPress = getValueByKeyName(vyBackwardKey_, msg); /* Y轴后退按键是否按下 */
    /* X轴按键逻辑处理 */
    /* 同时按下或者全部松开刹车 */
    if (!(isVxForwardPress ^ isVxBackwardPress)) {
        vx_ = 0;
        pressVxButtonTime_ = pressVxButtonTime_.fromSec(0);
        vxKeyState_ = KeyState::NONE;
    } else if (isVxForwardPress || isVxBackwardPress) {
        KeyState keyState = isVxForwardPress ? KeyState::FORWARD : KeyState::BACKWARD;
        if (keyState != vxKeyState_) {
            pressVxButtonTime_ = ros::Time::now();
        }
        vxKeyState_ = keyState;
        if (isVxForwardPress) {
            vx_ = vxFunction_->compute((ros::Time::now() - pressVxButtonTime_).toSec());
        } else {
            vx_ = vxFunction_->compute(-(ros::Time::now() - pressVxButtonTime_).toSec());
        }
    }
    /* Y轴按键逻辑处理 */
    /* 同时按下或者全部松开刹车 */
    if (!(isVyForwardPress ^ isVyBackwardPress)) {
        vx_ = 0;
        pressVyButtonTime_ = pressVyButtonTime_.fromSec(0);
        vyKeyState_ = KeyState::NONE;
    } else if (isVyForwardPress || isVyBackwardPress) {
        KeyState keyState = isVyForwardPress ? KeyState::FORWARD : KeyState::BACKWARD;
        if (keyState != vxKeyState_) {
            pressVyButtonTime_ = ros::Time::now();
        }
        vyKeyState_ = keyState;
        if (isVyForwardPress) {
            vy_ = vxFunction_->compute((ros::Time::now() - pressVyButtonTime_).toSec());
        } else {
            vy_ = vxFunction_->compute(-(ros::Time::now() - pressVyButtonTime_).toSec());
        }
    }
    /* 加速按键 */
    isSpeedUp_ = getValueByKeyName(speedUpKey_, msg);
}

bool KeyboardChannel::init()
{
    /* 读取配置 */
    CONFIG_ASSERT("keyboard/vx/forward_key", nodeParam_.getParam("keyboard/vx/forward_key", vxForwardKey_) && checkKey(vxForwardKey_));
    vxForwardKey_ = boost::to_upper_copy(vxForwardKey_);
    CONFIG_ASSERT("keyboard/vx/backward_key", nodeParam_.getParam("keyboard/vx/backward_key", vxBackwardKey_) && checkKey(vxBackwardKey_));
    vxBackwardKey_ = boost::to_upper_copy(vxBackwardKey_);
    CONFIG_ASSERT("keyboard/vy/forward_key", nodeParam_.getParam("keyboard/vy/forward_key", vyForwardKey_) && checkKey(vyForwardKey_));
    vyForwardKey_ = boost::to_upper_copy(vyForwardKey_);
    CONFIG_ASSERT("keyboard/vy/backward_key", nodeParam_.getParam("keyboard/vy/backward_key", vyBackwardKey_) && checkKey(vyBackwardKey_));
    vyBackwardKey_ = boost::to_upper_copy(vyBackwardKey_);
    CONFIG_ASSERT("keyboard/speed_up_key", nodeParam_.getParam("keyboard/speed_up_key", speedUpKey_) && checkKey(speedUpKey_));
    speedUpKey_ = boost::to_upper_copy(speedUpKey_);
    CONFIG_ASSERT("keyboard/topic", nodeParam_.getParam("keyboard/topic", keyboardTopic_));
    /* 初始化函数类 */
    vxFunction_.reset(new robot_toolbox::FunctionTool(ros::NodeHandle("~keyboard/vx/function"), ros::NodeHandle("~keyboard/vx/function")));
    vyFunction_.reset(new robot_toolbox::FunctionTool(ros::NodeHandle("~keyboard/vy/function"), ros::NodeHandle("~keyboard/vy/function")));
    if (!vxFunction_->init() || !vyFunction_->init()) return false;
    /* 订阅 */
    keyboardSubscriber_ = node_.subscribe<rm_rc_controller::Keyboard>(keyboardTopic_, 1000, &KeyboardChannel::keyboardCallback, this);
    return true;
}

void KeyboardChannel::getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ros::Duration period, std::map<std::string, bool>& enableModules)
{
    if (isSpeedUp_) enableModules["supercap"] = true;
    vx += vx_;
    vy += vy_;
}

}  // namespace rm_control