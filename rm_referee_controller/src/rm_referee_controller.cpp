#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <robot_toolbox/tool.h>

#include "rm_referee_controller/rm_referee_controller.h"

namespace rm_referee_controller
{
RMRefereeController::RMRefereeController()
{
}

bool RMRefereeController::init(robot_interface::RMRefereeInterface *hw, ros::NodeHandle &node)
{
    /* 读取配置并注册发布者 */
    std::string handleName;
    node.param<double>("publish_rate", publishRate_, 100);
    node.param<std::string>("handle_name", handleName, "rm_referee");
    handle_ = hw->getHandle(handleName);
    /* 初始化发布者 */
    isOnlinePublisher_.reset(new realtime_tools::RealtimePublisher<robot_msgs::BoolStamped>(node, "data/online", 1000));
    gameStatusPublisher_.reset(new realtime_tools::RealtimePublisher<GameStatus>(node, "data/game_status", 1000));
    hpPublisher_.reset(new realtime_tools::RealtimePublisher<HP>(node, "data/hp", 1000));
    icraStatusPublisher_.reset(new realtime_tools::RealtimePublisher<ICRAStatus>(node, "data/icra_status", 1000));
    eventPublisher_.reset(new realtime_tools::RealtimePublisher<Event>(node, "data/event", 1000));
    dartRemainingTimePublisher_.reset(new realtime_tools::RealtimePublisher<robot_msgs::DurationStamped>(node, "data/dart_remaining_time", 1000));
    robotStatusPublisher_.reset(new realtime_tools::RealtimePublisher<RobotStatus>(node, "data/robot_status", 1000));
    powerHeatDataPublisher_.reset(new realtime_tools::RealtimePublisher<PowerHeat>(node, "data/power_heat", 1000));
    positionPublisher_.reset(new realtime_tools::RealtimePublisher<Position>(node, "data/position", 1000));
    buffPublisher_.reset(new realtime_tools::RealtimePublisher<Buff>(node, "data/buff", 1000));
    aerialAttackTimePublisher_.reset(new realtime_tools::RealtimePublisher<robot_msgs::DurationStamped>(node, "data/aerial_attack_time", 1000));
    bulletRemainingStatusPublisher_.reset(new realtime_tools::RealtimePublisher<BulletRemainingStatus>(node, "data/bullet_remaining_status", 1000));
    rfidStatusPublisher_.reset(new realtime_tools::RealtimePublisher<RFIDStatus>(node, "data/rfid_status", 1000));
    dartClientCMDPublisher_.reset(new realtime_tools::RealtimePublisher<DartClientCMD>(node, "data/dart_client_cmd", 1000));
    gameResultPublisher_.reset(new realtime_tools::RealtimePublisher<GameResult>(node, "data/game_result", 1000));
    dartLaunchPublisher_.reset(new realtime_tools::RealtimePublisher<DartLaunch>(node, "data/dart_launch", 1000));
    supplyProjectileActionPublisher_.reset(new realtime_tools::RealtimePublisher<SupplyProjectileAction>(node, "data/supply_projectile_action", 1000));
    refereeWarningPublisher_.reset(new realtime_tools::RealtimePublisher<RefereeWarning>(node, "data/referee_warning", 1000));
    hurtPublisher_.reset(new realtime_tools::RealtimePublisher<Hurt>(node, "data/hurt", 1000));
    shootPublisher_.reset(new realtime_tools::RealtimePublisher<Shoot>(node, "data/shoot", 1000));
    interactivePublisher_.reset(new realtime_tools::RealtimePublisher<Interactive>(node, "data/interactive/receive", 1000));
    customControllerPublisher_.reset(new realtime_tools::RealtimePublisher<CustomController>(node, "data/custom_controller", 1000));
    /* 订阅 */
    interactiveSubscriber_ = node.subscribe<Interactive>("interactive/send", 1000, &RMRefereeController::interactiveCallback, this);
    deleteUISubscriber_    = node.subscribe<DeleteUI>("ui/delete", 1000, &RMRefereeController::deleteUICallback, this);
    updateUISubscriber_    = node.subscribe<UIData>("ui/update", 1000, &RMRefereeController::updateUICallback, this);
    /* 注册事件回调 */
    handle_.setGameResultCallback(boost::bind(&RMRefereeController::gameResultCallback, this, _1));
    handle_.setDartLaunchCallback(boost::bind(&RMRefereeController::dartLaunchCallback, this, _1, _2));
    handle_.setSupplyProjectileActionCallback(boost::bind(&RMRefereeController::supplyProjectileActionCallback, this, _1, _2, _3, _4, _5));
    handle_.setRefereeWarningCallback(boost::bind(&RMRefereeController::refereeWarningCallback, this, _1, _2));
    handle_.setHurtCallback(boost::bind(&RMRefereeController::hurtCallback, this, _1, _2));
    handle_.setShootCallback(boost::bind(&RMRefereeController::shootCallback, this, _1, _2, _3, _4));
    handle_.setInteractiveCallback(boost::bind(&RMRefereeController::interactiveCallback, this, _1, _2, _3));
    handle_.setCustomControllerCallback(boost::bind(&RMRefereeController::customControllerCallback, this, _1));
    ROS_INFO("RoboMaster Referee Controller started!");
    return true;
}

void RMRefereeController::gameResultCallback(robot_interface::RMRefereeHandle::GameResult result)
{
    if (gameResultPublisher_ && gameResultPublisher_->trylock()) {
        gameResultPublisher_->msg_.header.seq++;
        gameResultPublisher_->msg_.header.stamp = ros::Time::now();
        gameResultPublisher_->msg_.result       = static_cast<uint8_t>(result);
        gameResultPublisher_->unlockAndPublish();
    }
}

void RMRefereeController::dartLaunchCallback(robot_interface::RMRefereeHandle::GameGroup group, ros::Duration remainingTime)
{
    if (dartLaunchPublisher_ && dartLaunchPublisher_->trylock()) {
        dartLaunchPublisher_->msg_.header.seq++;
        dartLaunchPublisher_->msg_.header.stamp  = ros::Time::now();
        dartLaunchPublisher_->msg_.group         = static_cast<uint8_t>(group);
        dartLaunchPublisher_->msg_.remainingTime = remainingTime;
        dartLaunchPublisher_->unlockAndPublish();
    }
}

void RMRefereeController::supplyProjectileActionCallback(int supplyID, bool isAnyRobotSupply, robot_interface::RMRefereeHandle::RobotType robotType, robot_interface::RMRefereeHandle::SupplyProjectileStep step, int projectileNumber)
{
    if (supplyProjectileActionPublisher_ && supplyProjectileActionPublisher_->trylock()) {
        supplyProjectileActionPublisher_->msg_.header.seq++;
        supplyProjectileActionPublisher_->msg_.header.stamp     = ros::Time::now();
        supplyProjectileActionPublisher_->msg_.supplyID         = supplyID;
        supplyProjectileActionPublisher_->msg_.isAnyRobotSupply = isAnyRobotSupply;
        supplyProjectileActionPublisher_->msg_.robotType        = static_cast<uint8_t>(robotType);
        supplyProjectileActionPublisher_->msg_.step             = static_cast<uint8_t>(step);
        supplyProjectileActionPublisher_->msg_.projectileNumber = projectileNumber;
        supplyProjectileActionPublisher_->unlockAndPublish();
    }
}

void RMRefereeController::refereeWarningCallback(robot_interface::RMRefereeHandle::RefereeWarning level, robot_interface::RMRefereeHandle::RobotType robotType)
{
    if (refereeWarningPublisher_ && refereeWarningPublisher_->trylock()) {
        refereeWarningPublisher_->msg_.header.seq++;
        refereeWarningPublisher_->msg_.header.stamp = ros::Time::now();
        refereeWarningPublisher_->msg_.level        = static_cast<uint8_t>(level);
        refereeWarningPublisher_->msg_.robotType    = static_cast<uint8_t>(robotType);
        refereeWarningPublisher_->unlockAndPublish();
    }
}

void RMRefereeController::hurtCallback(int armorID, robot_interface::RMRefereeHandle::HurtType type)
{
    if (hurtPublisher_ && hurtPublisher_->trylock()) {
        hurtPublisher_->msg_.header.seq++;
        hurtPublisher_->msg_.header.stamp = ros::Time::now();
        hurtPublisher_->msg_.armorID      = armorID;
        hurtPublisher_->msg_.type         = static_cast<uint8_t>(type);
        hurtPublisher_->unlockAndPublish();
    }
}

void RMRefereeController::shootCallback(robot_interface::RMRefereeHandle::BulletType bulletType, robot_interface::RMRefereeHandle::ShooterType shooterType, int hz, double speed)
{
    if (shootPublisher_ && shootPublisher_->trylock()) {
        shootPublisher_->msg_.header.seq++;
        shootPublisher_->msg_.header.stamp = ros::Time::now();
        shootPublisher_->msg_.bulletType   = static_cast<uint8_t>(bulletType);
        shootPublisher_->msg_.shooterID    = static_cast<uint8_t>(shooterType);
        shootPublisher_->msg_.hz           = hz;
        shootPublisher_->msg_.speed        = speed;
        shootPublisher_->unlockAndPublish();
    }
}

void RMRefereeController::interactiveCallback(int cmdID, int senderID, std::vector<uint8_t> &data)
{
    if (interactivePublisher_ && interactivePublisher_->trylock()) {
        interactivePublisher_->msg_.header.seq++;
        interactivePublisher_->msg_.header.stamp = ros::Time::now();
        interactivePublisher_->msg_.data         = data;
        interactivePublisher_->msg_.cmdID        = cmdID;
        interactivePublisher_->msg_.id           = senderID;
        interactivePublisher_->unlockAndPublish();
    }
}

void RMRefereeController::customControllerCallback(std::vector<uint8_t> &data)
{
    if (customControllerPublisher_ && customControllerPublisher_->trylock()) {
        customControllerPublisher_->msg_.header.seq++;
        customControllerPublisher_->msg_.header.stamp = ros::Time::now();
        customControllerPublisher_->msg_.data         = data;
        customControllerPublisher_->unlockAndPublish();
    }
}

void RMRefereeController::update(const ros::Time &time, const ros::Duration &period)
{
    lastPublishDuration_ += period;
    if (lastPublishDuration_.toSec() >= 1 / publishRate_) {
        lastPublishDuration_ = ros::Duration(0);
        /* 发布裁判系统信息 */
        if (isOnlinePublisher_ && isOnlinePublisher_->trylock()) {
            isOnlinePublisher_->msg_.header.seq++;
            isOnlinePublisher_->msg_.header.stamp = time;
            isOnlinePublisher_->msg_.result       = handle_.getDataPtr()->isOnline;
            isOnlinePublisher_->unlockAndPublish();
        }
        if (dartRemainingTimePublisher_ && dartRemainingTimePublisher_->trylock()) {
            dartRemainingTimePublisher_->msg_.header.seq++;
            dartRemainingTimePublisher_->msg_.header.stamp = time;
            dartRemainingTimePublisher_->msg_.result       = handle_.getDataPtr()->dartRemainingTime;
            dartRemainingTimePublisher_->unlockAndPublish();
        }
        if (aerialAttackTimePublisher_ && aerialAttackTimePublisher_->trylock()) {
            aerialAttackTimePublisher_->msg_.header.seq++;
            aerialAttackTimePublisher_->msg_.header.stamp = time;
            aerialAttackTimePublisher_->msg_.result       = handle_.getDataPtr()->aerialAttackTime;
            aerialAttackTimePublisher_->unlockAndPublish();
        }
        if (gameStatusPublisher_ && gameStatusPublisher_->trylock()) {
            gameStatusPublisher_->msg_.header.seq++;
            gameStatusPublisher_->msg_.header.stamp    = time;
            gameStatusPublisher_->msg_.type            = static_cast<uint8_t>(handle_.getDataPtr()->gameStatus.type);
            gameStatusPublisher_->msg_.process         = static_cast<uint8_t>(handle_.getDataPtr()->gameStatus.process);
            gameStatusPublisher_->msg_.stageRemainTime = handle_.getDataPtr()->gameStatus.stageRemainTime;
            gameStatusPublisher_->msg_.syncTime        = handle_.getDataPtr()->gameStatus.syncTime;
            gameStatusPublisher_->unlockAndPublish();
        }
        if (hpPublisher_ && hpPublisher_->trylock()) {
            hpPublisher_->msg_.header.seq++;
            hpPublisher_->msg_.header.stamp   = time;
            hpPublisher_->msg_.red.hero1      = handle_.getDataPtr()->hp.red.hero1;
            hpPublisher_->msg_.red.engineer2  = handle_.getDataPtr()->hp.red.engineer2;
            hpPublisher_->msg_.red.standard3  = handle_.getDataPtr()->hp.red.standard3;
            hpPublisher_->msg_.red.standard4  = handle_.getDataPtr()->hp.red.standard4;
            hpPublisher_->msg_.red.standard5  = handle_.getDataPtr()->hp.red.standard5;
            hpPublisher_->msg_.red.sentry7    = handle_.getDataPtr()->hp.red.sentry7;
            hpPublisher_->msg_.red.outpost    = handle_.getDataPtr()->hp.red.outpost;
            hpPublisher_->msg_.red.base       = handle_.getDataPtr()->hp.red.base;
            hpPublisher_->msg_.blue.hero1     = handle_.getDataPtr()->hp.blue.hero1;
            hpPublisher_->msg_.blue.engineer2 = handle_.getDataPtr()->hp.blue.engineer2;
            hpPublisher_->msg_.blue.standard3 = handle_.getDataPtr()->hp.blue.standard3;
            hpPublisher_->msg_.blue.standard4 = handle_.getDataPtr()->hp.blue.standard4;
            hpPublisher_->msg_.blue.standard5 = handle_.getDataPtr()->hp.blue.standard5;
            hpPublisher_->msg_.blue.sentry7   = handle_.getDataPtr()->hp.blue.sentry7;
            hpPublisher_->msg_.blue.outpost   = handle_.getDataPtr()->hp.blue.outpost;
            hpPublisher_->msg_.blue.base      = handle_.getDataPtr()->hp.blue.base;
            hpPublisher_->unlockAndPublish();
        }
        if (icraStatusPublisher_ && icraStatusPublisher_->trylock()) {
            icraStatusPublisher_->msg_.header.seq++;
            icraStatusPublisher_->msg_.header.stamp          = time;
            icraStatusPublisher_->msg_.buff.f1.isActive      = handle_.getDataPtr()->icraStatus.buff.f1.isActive;
            icraStatusPublisher_->msg_.buff.f1.type          = static_cast<uint8_t>(handle_.getDataPtr()->icraStatus.buff.f1.type);
            icraStatusPublisher_->msg_.buff.f2.isActive      = handle_.getDataPtr()->icraStatus.buff.f2.isActive;
            icraStatusPublisher_->msg_.buff.f2.type          = static_cast<uint8_t>(handle_.getDataPtr()->icraStatus.buff.f2.type);
            icraStatusPublisher_->msg_.buff.f3.isActive      = handle_.getDataPtr()->icraStatus.buff.f3.isActive;
            icraStatusPublisher_->msg_.buff.f3.type          = static_cast<uint8_t>(handle_.getDataPtr()->icraStatus.buff.f3.type);
            icraStatusPublisher_->msg_.buff.f4.isActive      = handle_.getDataPtr()->icraStatus.buff.f4.isActive;
            icraStatusPublisher_->msg_.buff.f4.type          = static_cast<uint8_t>(handle_.getDataPtr()->icraStatus.buff.f4.type);
            icraStatusPublisher_->msg_.buff.f5.isActive      = handle_.getDataPtr()->icraStatus.buff.f5.isActive;
            icraStatusPublisher_->msg_.buff.f5.type          = static_cast<uint8_t>(handle_.getDataPtr()->icraStatus.buff.f5.type);
            icraStatusPublisher_->msg_.buff.f6.isActive      = handle_.getDataPtr()->icraStatus.buff.f6.isActive;
            icraStatusPublisher_->msg_.buff.f6.type          = static_cast<uint8_t>(handle_.getDataPtr()->icraStatus.buff.f6.type);
            icraStatusPublisher_->msg_.remainingBullet.red1  = handle_.getDataPtr()->icraStatus.remainingBullet.red1;
            icraStatusPublisher_->msg_.remainingBullet.red2  = handle_.getDataPtr()->icraStatus.remainingBullet.red2;
            icraStatusPublisher_->msg_.remainingBullet.blue1 = handle_.getDataPtr()->icraStatus.remainingBullet.blue1;
            icraStatusPublisher_->msg_.remainingBullet.blue2 = handle_.getDataPtr()->icraStatus.remainingBullet.blue2;
            icraStatusPublisher_->unlockAndPublish();
        }
        if (eventPublisher_ && eventPublisher_->trylock()) {
            eventPublisher_->msg_.header.seq++;
            eventPublisher_->msg_.header.stamp                    = time;
            eventPublisher_->msg_.restoration.isFirstOccupied     = handle_.getDataPtr()->event.restoration.isFirstOccupied;
            eventPublisher_->msg_.restoration.isSecondOccupied    = handle_.getDataPtr()->event.restoration.isSecondOccupied;
            eventPublisher_->msg_.restoration.isThirdOccupied     = handle_.getDataPtr()->event.restoration.isThirdOccupied;
            eventPublisher_->msg_.powerRune.isAttackPointOccupied = handle_.getDataPtr()->event.powerRune.isAttackPointOccupied;
            eventPublisher_->msg_.powerRune.isSmallActive         = handle_.getDataPtr()->event.powerRune.isSmallActive;
            eventPublisher_->msg_.powerRune.isBigActive           = handle_.getDataPtr()->event.powerRune.isBigActive;
            eventPublisher_->msg_.elevated.is2Occupied            = handle_.getDataPtr()->event.elevated.is2Occupied;
            eventPublisher_->msg_.elevated.is3Occupied            = handle_.getDataPtr()->event.elevated.is3Occupied;
            eventPublisher_->msg_.elevated.is4Occupied            = handle_.getDataPtr()->event.elevated.is4Occupied;
            eventPublisher_->msg_.isVirtualShieldActive           = handle_.getDataPtr()->event.isVirtualShieldActive;
            eventPublisher_->msg_.isOutpostActive                 = handle_.getDataPtr()->event.isOutpostActive;
            eventPublisher_->unlockAndPublish();
        }
        if (robotStatusPublisher_ && robotStatusPublisher_->trylock()) {
            robotStatusPublisher_->msg_.header.seq++;
            robotStatusPublisher_->msg_.header.stamp                    = time;
            robotStatusPublisher_->msg_.group                           = static_cast<uint8_t>(handle_.getDataPtr()->robotStatus.group);
            robotStatusPublisher_->msg_.type                            = static_cast<uint8_t>(handle_.getDataPtr()->robotStatus.type);
            robotStatusPublisher_->msg_.level                           = handle_.getDataPtr()->robotStatus.level;
            robotStatusPublisher_->msg_.hp                              = handle_.getDataPtr()->robotStatus.hp;
            robotStatusPublisher_->msg_.maxHP                           = handle_.getDataPtr()->robotStatus.maxHP;
            robotStatusPublisher_->msg_.shooter.first17mm.coolingRate   = handle_.getDataPtr()->robotStatus.shooter.first17mm.coolingRate;
            robotStatusPublisher_->msg_.shooter.first17mm.coolingLimit  = handle_.getDataPtr()->robotStatus.shooter.first17mm.coolingLimit;
            robotStatusPublisher_->msg_.shooter.first17mm.speedLimit    = handle_.getDataPtr()->robotStatus.shooter.first17mm.speedLimit;
            robotStatusPublisher_->msg_.shooter.second17mm.coolingRate  = handle_.getDataPtr()->robotStatus.shooter.second17mm.coolingRate;
            robotStatusPublisher_->msg_.shooter.second17mm.coolingLimit = handle_.getDataPtr()->robotStatus.shooter.second17mm.coolingLimit;
            robotStatusPublisher_->msg_.shooter.second17mm.speedLimit   = handle_.getDataPtr()->robotStatus.shooter.second17mm.speedLimit;
            robotStatusPublisher_->msg_.shooter.first42mm.coolingRate   = handle_.getDataPtr()->robotStatus.shooter.first42mm.coolingRate;
            robotStatusPublisher_->msg_.shooter.first42mm.coolingLimit  = handle_.getDataPtr()->robotStatus.shooter.first42mm.coolingLimit;
            robotStatusPublisher_->msg_.shooter.first42mm.speedLimit    = handle_.getDataPtr()->robotStatus.shooter.first42mm.speedLimit;
            robotStatusPublisher_->msg_.chassisPowerLimit               = handle_.getDataPtr()->robotStatus.chassisPowerLimit;
            robotStatusPublisher_->msg_.power.gimbal                    = handle_.getDataPtr()->robotStatus.power.gimbal;
            robotStatusPublisher_->msg_.power.shooter                   = handle_.getDataPtr()->robotStatus.power.shooter;
            robotStatusPublisher_->msg_.power.chassis                   = handle_.getDataPtr()->robotStatus.power.chassis;
            robotStatusPublisher_->unlockAndPublish();
        }
        if (powerHeatDataPublisher_ && powerHeatDataPublisher_->trylock()) {
            powerHeatDataPublisher_->msg_.header.seq++;
            powerHeatDataPublisher_->msg_.header.stamp           = time;
            powerHeatDataPublisher_->msg_.voltage                = handle_.getDataPtr()->powerHeatData.voltage;
            powerHeatDataPublisher_->msg_.current                = handle_.getDataPtr()->powerHeatData.current;
            powerHeatDataPublisher_->msg_.power                  = handle_.getDataPtr()->powerHeatData.power;
            powerHeatDataPublisher_->msg_.buffer                 = handle_.getDataPtr()->powerHeatData.buffer;
            powerHeatDataPublisher_->msg_.shooterHeat.first17mm  = handle_.getDataPtr()->powerHeatData.shooterHeat.first17mm;
            powerHeatDataPublisher_->msg_.shooterHeat.second17mm = handle_.getDataPtr()->powerHeatData.shooterHeat.second17mm;
            powerHeatDataPublisher_->msg_.shooterHeat.first42mm  = handle_.getDataPtr()->powerHeatData.shooterHeat.first42mm;
            powerHeatDataPublisher_->unlockAndPublish();
        }
        if (positionPublisher_ && positionPublisher_->trylock()) {
            positionPublisher_->msg_.header.seq++;
            positionPublisher_->msg_.header.stamp = time;
            positionPublisher_->msg_.x            = handle_.getDataPtr()->position.x;
            positionPublisher_->msg_.y            = handle_.getDataPtr()->position.y;
            positionPublisher_->msg_.z            = handle_.getDataPtr()->position.z;
            positionPublisher_->msg_.shooterYaw   = handle_.getDataPtr()->position.shooterYaw;
            positionPublisher_->unlockAndPublish();
        }
        if (buffPublisher_ && buffPublisher_->trylock()) {
            buffPublisher_->msg_.header.seq++;
            buffPublisher_->msg_.header.stamp           = time;
            buffPublisher_->msg_.isHealing              = handle_.getDataPtr()->buff.isHealing;
            buffPublisher_->msg_.isShooterCoolingFaster = handle_.getDataPtr()->buff.isShooterCoolingFaster;
            buffPublisher_->msg_.isDefenseBuff          = handle_.getDataPtr()->buff.isDefenseBuff;
            buffPublisher_->msg_.isAttackBuff           = handle_.getDataPtr()->buff.isAttackBuff;
            buffPublisher_->unlockAndPublish();
        }
        if (bulletRemainingStatusPublisher_ && bulletRemainingStatusPublisher_->trylock()) {
            bulletRemainingStatusPublisher_->msg_.header.seq++;
            bulletRemainingStatusPublisher_->msg_.header.stamp        = time;
            bulletRemainingStatusPublisher_->msg_.all17mmRemainingNum = handle_.getDataPtr()->bulletRemaininggStatus.all17mmRemainingNum;
            bulletRemainingStatusPublisher_->msg_.all42mmRemainingNum = handle_.getDataPtr()->bulletRemaininggStatus.all42mmRemainingNum;
            bulletRemainingStatusPublisher_->msg_.remainingCoin       = handle_.getDataPtr()->bulletRemaininggStatus.remainingCoin;
            bulletRemainingStatusPublisher_->unlockAndPublish();
        }
        if (rfidStatusPublisher_ && rfidStatusPublisher_->trylock()) {
            rfidStatusPublisher_->msg_.header.seq++;
            rfidStatusPublisher_->msg_.header.stamp = time;
            rfidStatusPublisher_->msg_.base         = handle_.getDataPtr()->rfidStatus.base;
            rfidStatusPublisher_->msg_.elevated     = handle_.getDataPtr()->rfidStatus.elevated;
            rfidStatusPublisher_->msg_.powerRune    = handle_.getDataPtr()->rfidStatus.powerRune;
            rfidStatusPublisher_->msg_.launchRamp   = handle_.getDataPtr()->rfidStatus.launchRamp;
            rfidStatusPublisher_->msg_.outpost      = handle_.getDataPtr()->rfidStatus.outpost;
            rfidStatusPublisher_->msg_.restoration  = handle_.getDataPtr()->rfidStatus.restoration;
            rfidStatusPublisher_->msg_.engineer     = handle_.getDataPtr()->rfidStatus.engineer;
            rfidStatusPublisher_->unlockAndPublish();
        }
        if (dartClientCMDPublisher_ && dartClientCMDPublisher_->trylock()) {
            dartClientCMDPublisher_->msg_.header.seq++;
            dartClientCMDPublisher_->msg_.header.stamp      = time;
            dartClientCMDPublisher_->msg_.status            = static_cast<uint8_t>(handle_.getDataPtr()->dartClientCMD.status);
            dartClientCMDPublisher_->msg_.target            = static_cast<uint8_t>(handle_.getDataPtr()->dartClientCMD.target);
            dartClientCMDPublisher_->msg_.targetChangeTime  = handle_.getDataPtr()->dartClientCMD.targetChangeTime;
            dartClientCMDPublisher_->msg_.speed.first       = handle_.getDataPtr()->dartClientCMD.speed.first;
            dartClientCMDPublisher_->msg_.speed.second      = handle_.getDataPtr()->dartClientCMD.speed.second;
            dartClientCMDPublisher_->msg_.speed.third       = handle_.getDataPtr()->dartClientCMD.speed.third;
            dartClientCMDPublisher_->msg_.speed.fourth      = handle_.getDataPtr()->dartClientCMD.speed.fourth;
            dartClientCMDPublisher_->msg_.lastLaunchTime    = handle_.getDataPtr()->dartClientCMD.lastLaunchTime;
            dartClientCMDPublisher_->msg_.operateLaunchTime = handle_.getDataPtr()->dartClientCMD.operateLaunchTime;
            dartClientCMDPublisher_->unlockAndPublish();
        }
    }
    /* 发布UI信息 */
    std::vector<robot_interface::RMRefereeHandle::UIData> publishData;
    std::vector<UIInfo *> editData;
    std::vector<robot_interface::RMRefereeHandle::UIData> publishStringData;
    std::vector<UIInfo *> editStringData;
    for (size_t i = 0; i < uiWidgets_.size(); i++) {
        for (auto iter = uiWidgets_[i].begin(); iter != uiWidgets_[i].end(); ++iter) {
            if (uiWidgets_[i].find(iter->first) == uiWidgets_[i].end()) break;
            bool isDiff = false;
            if (iter->second.oldProperty.name != iter->second.property.name)
                isDiff = true;
            else if (iter->second.oldProperty.method != iter->second.property.method)
                isDiff = true;
            else if (iter->second.oldProperty.layer != iter->second.property.layer)
                isDiff = true;
            else if (iter->second.oldProperty.type != iter->second.property.type)
                isDiff = true;
            else if (iter->second.oldProperty.line.color != iter->second.property.line.color)
                isDiff = true;
            else if (iter->second.oldProperty.line.width != iter->second.property.line.width)
                isDiff = true;
            else if (iter->second.oldProperty.line.start.x != iter->second.property.line.start.x)
                isDiff = true;
            else if (iter->second.oldProperty.line.start.y != iter->second.property.line.start.y)
                isDiff = true;
            else if (iter->second.oldProperty.line.end.x != iter->second.property.line.end.x)
                isDiff = true;
            else if (iter->second.oldProperty.line.end.y != iter->second.property.line.end.y)
                isDiff = true;
            else if (iter->second.oldProperty.rectangle.color != iter->second.property.rectangle.color)
                isDiff = true;
            else if (iter->second.oldProperty.rectangle.width != iter->second.property.rectangle.width)
                isDiff = true;
            else if (iter->second.oldProperty.rectangle.start.x != iter->second.property.rectangle.start.x)
                isDiff = true;
            else if (iter->second.oldProperty.rectangle.start.y != iter->second.property.rectangle.start.y)
                isDiff = true;
            else if (iter->second.oldProperty.rectangle.end.x != iter->second.property.rectangle.end.x)
                isDiff = true;
            else if (iter->second.oldProperty.rectangle.end.y != iter->second.property.rectangle.end.y)
                isDiff = true;
            else if (iter->second.oldProperty.circle.color != iter->second.property.circle.color)
                isDiff = true;
            else if (iter->second.oldProperty.circle.width != iter->second.property.circle.width)
                isDiff = true;
            else if (iter->second.oldProperty.circle.center.x != iter->second.property.circle.center.x)
                isDiff = true;
            else if (iter->second.oldProperty.circle.center.y != iter->second.property.circle.center.y)
                isDiff = true;
            else if (iter->second.oldProperty.circle.radius != iter->second.property.circle.radius)
                isDiff = true;
            else if (iter->second.oldProperty.oval.color != iter->second.property.oval.color)
                isDiff = true;
            else if (iter->second.oldProperty.oval.width != iter->second.property.oval.width)
                isDiff = true;
            else if (iter->second.oldProperty.oval.center.x != iter->second.property.oval.center.x)
                isDiff = true;
            else if (iter->second.oldProperty.oval.center.y != iter->second.property.oval.center.y)
                isDiff = true;
            else if (iter->second.oldProperty.oval.xRadius != iter->second.property.oval.xRadius)
                isDiff = true;
            else if (iter->second.oldProperty.oval.yRadius != iter->second.property.oval.yRadius)
                isDiff = true;
            else if (iter->second.oldProperty.arc.color != iter->second.property.arc.color)
                isDiff = true;
            else if (iter->second.oldProperty.arc.width != iter->second.property.arc.width)
                isDiff = true;
            else if (iter->second.oldProperty.arc.angle.start != iter->second.property.arc.angle.start)
                isDiff = true;
            else if (iter->second.oldProperty.arc.angle.end != iter->second.property.arc.angle.end)
                isDiff = true;
            else if (iter->second.oldProperty.arc.center.x != iter->second.property.arc.center.x)
                isDiff = true;
            else if (iter->second.oldProperty.arc.center.y != iter->second.property.arc.center.y)
                isDiff = true;
            else if (iter->second.oldProperty.arc.xRadius != iter->second.property.arc.xRadius)
                isDiff = true;
            else if (iter->second.oldProperty.arc.yRadius != iter->second.property.arc.yRadius)
                isDiff = true;
            else if (iter->second.oldProperty.floatNumber.color != iter->second.property.floatNumber.color)
                isDiff = true;
            else if (iter->second.oldProperty.floatNumber.width != iter->second.property.floatNumber.width)
                isDiff = true;
            else if (iter->second.oldProperty.floatNumber.start.x != iter->second.property.floatNumber.start.x)
                isDiff = true;
            else if (iter->second.oldProperty.floatNumber.start.y != iter->second.property.floatNumber.start.y)
                isDiff = true;
            else if (iter->second.oldProperty.floatNumber.data != iter->second.property.floatNumber.data)
                isDiff = true;
            else if (iter->second.oldProperty.floatNumber.fontSize != iter->second.property.floatNumber.fontSize)
                isDiff = true;
            else if (iter->second.oldProperty.floatNumber.digits != iter->second.property.floatNumber.digits)
                isDiff = true;
            else if (iter->second.oldProperty.intNumber.color != iter->second.property.intNumber.color)
                isDiff = true;
            else if (iter->second.oldProperty.intNumber.width != iter->second.property.intNumber.width)
                isDiff = true;
            else if (iter->second.oldProperty.intNumber.start.x != iter->second.property.intNumber.start.x)
                isDiff = true;
            else if (iter->second.oldProperty.intNumber.start.y != iter->second.property.intNumber.start.y)
                isDiff = true;
            else if (iter->second.oldProperty.intNumber.data != iter->second.property.intNumber.data)
                isDiff = true;
            else if (iter->second.oldProperty.intNumber.fontSize != iter->second.property.intNumber.fontSize)
                isDiff = true;
            else if (iter->second.oldProperty.string.color != iter->second.property.string.color)
                isDiff = true;
            else if (iter->second.oldProperty.string.width != iter->second.property.string.width)
                isDiff = true;
            else if (iter->second.oldProperty.string.start.x != iter->second.property.string.start.x)
                isDiff = true;
            else if (iter->second.oldProperty.string.start.y != iter->second.property.string.start.y)
                isDiff = true;
            else if (iter->second.oldProperty.string.data != iter->second.property.string.data)
                isDiff = true;
            else if (iter->second.oldProperty.string.fontSize != iter->second.property.string.fontSize)
                isDiff = true;
            if (iter->second.forceUpdate || isDiff) {
                iter->second.forceUpdate = false;
                if (iter->second.property.type == robot_interface::RMRefereeHandle::GraphType::STRING) {
                    editStringData.push_back(&uiWidgets_[i][iter->first]);
                    publishStringData.push_back(iter->second.property);
                } else {
                    editData.push_back(&uiWidgets_[i][iter->first]);
                    publishData.push_back(uiWidgets_[i][iter->first].property);
                }
            }
        }
    }
    size_t count = handle_.callSendGraphUIFunction(publishData);
    for (size_t i = 0; i < count; i++) {
        editData[i]->forceUpdate = false;
        editData[i]->isFirstSend = false;
        editData[i]->oldProperty = editData[i]->property;
    }
    count = handle_.callSendGraphUIStringFunction(publishStringData);
    for (size_t i = 0; i < count; i++) {
        editStringData[i]->forceUpdate = false;
        editStringData[i]->isFirstSend = false;
        editStringData[i]->oldProperty = editStringData[i]->property;
    }
}

void RMRefereeController::interactiveCallback(const InteractiveConstPtr &msg)
{
    if (msg->id > 9 || msg->id < 1) {
        ROS_ERROR("Interactive message send failed, because Robot Type error!");
        return;
    }
    std::vector<uint8_t> data = msg->data;
    if (!handle_.callSendInteractiveFunction(msg->cmdID, static_cast<robot_interface::RMRefereeHandle::RobotType>(msg->id), data)) {
        ROS_ERROR("Send Interactive message error.");
    }
}

void RMRefereeController::deleteUICallback(const DeleteUIConstPtr &msg)
{
    if (msg->layer > 9 || msg->layer < 0) {
        ROS_ERROR("Delete UI message send failed, because layer error!");
        return;
    }
    robot_interface::RMRefereeHandle::GraphDeleteCMD cmd = static_cast<robot_interface::RMRefereeHandle::GraphDeleteCMD>(msg->cmd);
    if (!handle_.callDeleteLayerUIFunction(cmd, msg->layer)) {
        ROS_ERROR("Send Delete UI message error.");
        return;
    }
    /* 清除指定层元素 */
    if (cmd == robot_interface::RMRefereeHandle::GraphDeleteCMD::ALL) {
        for (size_t i = 0; i < uiWidgets_.size(); i++) {
            std::map<int, UIInfo>().swap(uiWidgets_[i]);
        }
    } else {
        std::map<int, UIInfo>().swap(uiWidgets_[msg->layer]);
    }
}

void RMRefereeController::updateUICallback(const UIDataConstPtr &msg)
{
    /* 转换UI数据 */
    for (size_t i = 0; i < msg->lines.size(); i++) {
        if (uiWidgets_[msg->layer].count(msg->lines[i].id) > 0 && !uiWidgets_[msg->layer][msg->lines[i].id].isFirstSend) {
            uiWidgets_[msg->layer][msg->lines[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::EDIT;
        } else {
            uiWidgets_[msg->layer][msg->lines[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::ADD;
            uiWidgets_[msg->layer][msg->lines[i].id].isFirstSend     = true;
        }
        if (msg->lines[i].hide) uiWidgets_[msg->layer][msg->lines[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::DELETE;
        uiWidgets_[msg->layer][msg->lines[i].id].property.layer = msg->layer;
        uiWidgets_[msg->layer][msg->lines[i].id].property.type  = robot_interface::RMRefereeHandle::GraphType::LINE;

        uiWidgets_[msg->layer][msg->lines[i].id].property.name         = msg->lines[i].id;
        uiWidgets_[msg->layer][msg->lines[i].id].forceUpdate           = msg->lines[i].isForceUpdate;
        uiWidgets_[msg->layer][msg->lines[i].id].property.line.color   = static_cast<robot_interface::RMRefereeHandle::GraphColor>(msg->lines[i].color);
        uiWidgets_[msg->layer][msg->lines[i].id].property.line.width   = msg->lines[i].width;
        uiWidgets_[msg->layer][msg->lines[i].id].property.line.start.x = msg->lines[i].startX;
        uiWidgets_[msg->layer][msg->lines[i].id].property.line.start.y = msg->lines[i].startY;
        uiWidgets_[msg->layer][msg->lines[i].id].property.line.end.x   = msg->lines[i].endX;
        uiWidgets_[msg->layer][msg->lines[i].id].property.line.end.y   = msg->lines[i].endY;
    }
    for (size_t i = 0; i < msg->rectangles.size(); i++) {
        if (uiWidgets_[msg->layer].count(msg->rectangles[i].id) > 0 && !uiWidgets_[msg->layer][msg->rectangles[i].id].isFirstSend) {
            uiWidgets_[msg->layer][msg->rectangles[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::EDIT;
        } else {
            uiWidgets_[msg->layer][msg->rectangles[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::ADD;
            uiWidgets_[msg->layer][msg->rectangles[i].id].isFirstSend     = true;
        }
        if (msg->rectangles[i].hide) uiWidgets_[msg->layer][msg->rectangles[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::DELETE;
        uiWidgets_[msg->layer][msg->rectangles[i].id].property.layer = msg->layer;
        uiWidgets_[msg->layer][msg->rectangles[i].id].property.type  = robot_interface::RMRefereeHandle::GraphType::RECTANGLE;

        uiWidgets_[msg->layer][msg->rectangles[i].id].property.name              = msg->rectangles[i].id;
        uiWidgets_[msg->layer][msg->rectangles[i].id].forceUpdate                = msg->rectangles[i].isForceUpdate;
        uiWidgets_[msg->layer][msg->rectangles[i].id].property.rectangle.color   = static_cast<robot_interface::RMRefereeHandle::GraphColor>(msg->rectangles[i].color);
        uiWidgets_[msg->layer][msg->rectangles[i].id].property.rectangle.width   = msg->rectangles[i].width;
        uiWidgets_[msg->layer][msg->rectangles[i].id].property.rectangle.start.x = msg->rectangles[i].startX;
        uiWidgets_[msg->layer][msg->rectangles[i].id].property.rectangle.start.y = msg->rectangles[i].startY;
        uiWidgets_[msg->layer][msg->rectangles[i].id].property.rectangle.end.x   = msg->rectangles[i].endX;
        uiWidgets_[msg->layer][msg->rectangles[i].id].property.rectangle.end.y   = msg->rectangles[i].endY;
    }
    for (size_t i = 0; i < msg->circles.size(); i++) {
        if (uiWidgets_[msg->layer].count(msg->circles[i].id) > 0 && !uiWidgets_[msg->layer][msg->circles[i].id].isFirstSend) {
            uiWidgets_[msg->layer][msg->circles[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::EDIT;
        } else {
            uiWidgets_[msg->layer][msg->circles[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::ADD;
            uiWidgets_[msg->layer][msg->circles[i].id].isFirstSend     = true;
        }
        if (msg->circles[i].hide) uiWidgets_[msg->layer][msg->circles[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::DELETE;
        uiWidgets_[msg->layer][msg->circles[i].id].property.layer = msg->layer;
        uiWidgets_[msg->layer][msg->circles[i].id].property.type  = robot_interface::RMRefereeHandle::GraphType::CIRCLE;

        uiWidgets_[msg->layer][msg->circles[i].id].property.name            = msg->circles[i].id;
        uiWidgets_[msg->layer][msg->circles[i].id].forceUpdate              = msg->circles[i].isForceUpdate;
        uiWidgets_[msg->layer][msg->circles[i].id].property.circle.color    = static_cast<robot_interface::RMRefereeHandle::GraphColor>(msg->circles[i].color);
        uiWidgets_[msg->layer][msg->circles[i].id].property.circle.width    = msg->circles[i].width;
        uiWidgets_[msg->layer][msg->circles[i].id].property.circle.center.x = msg->circles[i].centerX;
        uiWidgets_[msg->layer][msg->circles[i].id].property.circle.center.y = msg->circles[i].centerY;
        uiWidgets_[msg->layer][msg->circles[i].id].property.circle.radius   = msg->circles[i].radius;
    }
    for (size_t i = 0; i < msg->ovals.size(); i++) {
        if (uiWidgets_[msg->layer].count(msg->ovals[i].id) > 0 && !uiWidgets_[msg->layer][msg->ovals[i].id].isFirstSend) {
            uiWidgets_[msg->layer][msg->ovals[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::EDIT;
        } else {
            uiWidgets_[msg->layer][msg->ovals[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::ADD;
            uiWidgets_[msg->layer][msg->ovals[i].id].isFirstSend     = true;
        }
        if (msg->ovals[i].hide) uiWidgets_[msg->layer][msg->ovals[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::DELETE;
        uiWidgets_[msg->layer][msg->ovals[i].id].property.layer = msg->layer;
        uiWidgets_[msg->layer][msg->ovals[i].id].property.type  = robot_interface::RMRefereeHandle::GraphType::OVAL;

        uiWidgets_[msg->layer][msg->ovals[i].id].property.name          = msg->ovals[i].id;
        uiWidgets_[msg->layer][msg->ovals[i].id].forceUpdate            = msg->ovals[i].isForceUpdate;
        uiWidgets_[msg->layer][msg->ovals[i].id].property.oval.color    = static_cast<robot_interface::RMRefereeHandle::GraphColor>(msg->ovals[i].color);
        uiWidgets_[msg->layer][msg->ovals[i].id].property.oval.width    = msg->ovals[i].width;
        uiWidgets_[msg->layer][msg->ovals[i].id].property.oval.center.x = msg->ovals[i].centerX;
        uiWidgets_[msg->layer][msg->ovals[i].id].property.oval.center.y = msg->ovals[i].centerY;
        uiWidgets_[msg->layer][msg->ovals[i].id].property.oval.xRadius  = msg->ovals[i].xRadius;
        uiWidgets_[msg->layer][msg->ovals[i].id].property.oval.yRadius  = msg->ovals[i].yRadius;
    }
    for (size_t i = 0; i < msg->arcs.size(); i++) {
        if (uiWidgets_[msg->layer].count(msg->arcs[i].id) > 0 && !uiWidgets_[msg->layer][msg->arcs[i].id].isFirstSend) {
            uiWidgets_[msg->layer][msg->arcs[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::EDIT;
        } else {
            uiWidgets_[msg->layer][msg->arcs[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::ADD;
            uiWidgets_[msg->layer][msg->arcs[i].id].isFirstSend     = true;
        }
        if (msg->arcs[i].hide) uiWidgets_[msg->layer][msg->arcs[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::DELETE;
        uiWidgets_[msg->layer][msg->arcs[i].id].property.layer = msg->layer;
        uiWidgets_[msg->layer][msg->arcs[i].id].property.type  = robot_interface::RMRefereeHandle::GraphType::ARC;

        uiWidgets_[msg->layer][msg->arcs[i].id].property.name            = msg->arcs[i].id;
        uiWidgets_[msg->layer][msg->arcs[i].id].forceUpdate              = msg->arcs[i].isForceUpdate;
        uiWidgets_[msg->layer][msg->arcs[i].id].property.arc.color       = static_cast<robot_interface::RMRefereeHandle::GraphColor>(msg->arcs[i].color);
        uiWidgets_[msg->layer][msg->arcs[i].id].property.arc.width       = msg->arcs[i].width;
        uiWidgets_[msg->layer][msg->arcs[i].id].property.arc.angle.start = msg->arcs[i].startAngle;
        uiWidgets_[msg->layer][msg->arcs[i].id].property.arc.angle.end   = msg->arcs[i].endAngle;
        uiWidgets_[msg->layer][msg->arcs[i].id].property.arc.center.x    = msg->arcs[i].centerX;
        uiWidgets_[msg->layer][msg->arcs[i].id].property.arc.center.y    = msg->arcs[i].centerY;
        uiWidgets_[msg->layer][msg->arcs[i].id].property.arc.xRadius     = msg->arcs[i].xRadius;
        uiWidgets_[msg->layer][msg->arcs[i].id].property.arc.yRadius     = msg->arcs[i].yRadius;
    }
    for (size_t i = 0; i < msg->floatNumbers.size(); i++) {
        if (uiWidgets_[msg->layer].count(msg->floatNumbers[i].id) > 0 && !uiWidgets_[msg->layer][msg->floatNumbers[i].id].isFirstSend) {
            uiWidgets_[msg->layer][msg->floatNumbers[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::EDIT;
        } else {
            uiWidgets_[msg->layer][msg->floatNumbers[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::ADD;
            uiWidgets_[msg->layer][msg->floatNumbers[i].id].isFirstSend     = true;
        }
        if (msg->floatNumbers[i].hide) uiWidgets_[msg->layer][msg->floatNumbers[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::DELETE;
        uiWidgets_[msg->layer][msg->floatNumbers[i].id].property.layer = msg->layer;
        uiWidgets_[msg->layer][msg->floatNumbers[i].id].property.type  = robot_interface::RMRefereeHandle::GraphType::FLOAT_NUMBER;

        uiWidgets_[msg->layer][msg->floatNumbers[i].id].property.name                 = msg->floatNumbers[i].id;
        uiWidgets_[msg->layer][msg->floatNumbers[i].id].forceUpdate                   = msg->floatNumbers[i].isForceUpdate;
        uiWidgets_[msg->layer][msg->floatNumbers[i].id].property.floatNumber.color    = static_cast<robot_interface::RMRefereeHandle::GraphColor>(msg->floatNumbers[i].color);
        uiWidgets_[msg->layer][msg->floatNumbers[i].id].property.floatNumber.width    = msg->floatNumbers[i].width;
        uiWidgets_[msg->layer][msg->floatNumbers[i].id].property.floatNumber.start.x  = msg->floatNumbers[i].startX;
        uiWidgets_[msg->layer][msg->floatNumbers[i].id].property.floatNumber.start.y  = msg->floatNumbers[i].startY;
        uiWidgets_[msg->layer][msg->floatNumbers[i].id].property.floatNumber.data     = msg->floatNumbers[i].data;
        uiWidgets_[msg->layer][msg->floatNumbers[i].id].property.floatNumber.fontSize = msg->floatNumbers[i].fontSize;
        uiWidgets_[msg->layer][msg->floatNumbers[i].id].property.floatNumber.digits   = msg->floatNumbers[i].digits;
    }
    for (size_t i = 0; i < msg->intNumbers.size(); i++) {
        if (uiWidgets_[msg->layer].count(msg->intNumbers[i].id) > 0 && !uiWidgets_[msg->layer][msg->intNumbers[i].id].isFirstSend) {
            uiWidgets_[msg->layer][msg->intNumbers[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::EDIT;
        } else {
            uiWidgets_[msg->layer][msg->intNumbers[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::ADD;
            uiWidgets_[msg->layer][msg->intNumbers[i].id].isFirstSend     = true;
        }
        if (msg->intNumbers[i].hide) uiWidgets_[msg->layer][msg->intNumbers[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::DELETE;
        uiWidgets_[msg->layer][msg->intNumbers[i].id].property.layer = msg->layer;
        uiWidgets_[msg->layer][msg->intNumbers[i].id].property.type  = robot_interface::RMRefereeHandle::GraphType::INT_NUMBER;

        uiWidgets_[msg->layer][msg->intNumbers[i].id].property.name               = msg->intNumbers[i].id;
        uiWidgets_[msg->layer][msg->intNumbers[i].id].forceUpdate                 = msg->intNumbers[i].isForceUpdate;
        uiWidgets_[msg->layer][msg->intNumbers[i].id].property.intNumber.color    = static_cast<robot_interface::RMRefereeHandle::GraphColor>(msg->intNumbers[i].color);
        uiWidgets_[msg->layer][msg->intNumbers[i].id].property.intNumber.width    = msg->intNumbers[i].width;
        uiWidgets_[msg->layer][msg->intNumbers[i].id].property.intNumber.start.x  = msg->intNumbers[i].startX;
        uiWidgets_[msg->layer][msg->intNumbers[i].id].property.intNumber.start.y  = msg->intNumbers[i].startY;
        uiWidgets_[msg->layer][msg->intNumbers[i].id].property.intNumber.data     = msg->intNumbers[i].data;
        uiWidgets_[msg->layer][msg->intNumbers[i].id].property.intNumber.fontSize = msg->intNumbers[i].fontSize;
    }
    for (size_t i = 0; i < msg->strings.size(); i++) {
        if (uiWidgets_[msg->layer].count(msg->strings[i].id) > 0 && !uiWidgets_[msg->layer][msg->strings[i].id].isFirstSend) {
            uiWidgets_[msg->layer][msg->strings[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::EDIT;
        } else {
            uiWidgets_[msg->layer][msg->strings[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::ADD;
            uiWidgets_[msg->layer][msg->strings[i].id].isFirstSend     = true;
        }
        if (msg->strings[i].hide) uiWidgets_[msg->layer][msg->strings[i].id].property.method = robot_interface::RMRefereeHandle::GraphMethod::DELETE;
        uiWidgets_[msg->layer][msg->strings[i].id].property.layer = msg->layer;
        uiWidgets_[msg->layer][msg->strings[i].id].property.type  = robot_interface::RMRefereeHandle::GraphType::STRING;

        uiWidgets_[msg->layer][msg->strings[i].id].property.name            = msg->strings[i].id;
        uiWidgets_[msg->layer][msg->strings[i].id].forceUpdate              = msg->strings[i].isForceUpdate;
        uiWidgets_[msg->layer][msg->strings[i].id].property.string.color    = static_cast<robot_interface::RMRefereeHandle::GraphColor>(msg->strings[i].color);
        uiWidgets_[msg->layer][msg->strings[i].id].property.string.width    = msg->strings[i].width;
        uiWidgets_[msg->layer][msg->strings[i].id].property.string.start.x  = msg->strings[i].startX;
        uiWidgets_[msg->layer][msg->strings[i].id].property.string.start.y  = msg->strings[i].startY;
        uiWidgets_[msg->layer][msg->strings[i].id].property.string.data     = msg->strings[i].data;
        uiWidgets_[msg->layer][msg->strings[i].id].property.string.fontSize = msg->strings[i].fontSize;
    }
}

void RMRefereeController::starting(const ros::Time &time)
{
}

void RMRefereeController::stopping(const ros::Time &time)
{
}

}  // namespace rm_referee_controller

PLUGINLIB_EXPORT_CLASS(rm_referee_controller::RMRefereeController, controller_interface::ControllerBase);