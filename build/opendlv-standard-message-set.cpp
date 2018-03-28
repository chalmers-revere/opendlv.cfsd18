#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace sim {

int32_t Frame::ID() {
    return 1001;
}

const std::string Frame::ShortName() {
    return "Frame";
}
const std::string Frame::LongName() {
    return "opendlv.sim.Frame";
}

Frame& Frame::x(const float &v) noexcept {
    m_x = v;
    return *this;
}
float Frame::x() const noexcept {
    return m_x;
}

Frame& Frame::y(const float &v) noexcept {
    m_y = v;
    return *this;
}
float Frame::y() const noexcept {
    return m_y;
}

Frame& Frame::z(const float &v) noexcept {
    m_z = v;
    return *this;
}
float Frame::z() const noexcept {
    return m_z;
}

Frame& Frame::roll(const float &v) noexcept {
    m_roll = v;
    return *this;
}
float Frame::roll() const noexcept {
    return m_roll;
}

Frame& Frame::pitch(const float &v) noexcept {
    m_pitch = v;
    return *this;
}
float Frame::pitch() const noexcept {
    return m_pitch;
}

Frame& Frame::yaw(const float &v) noexcept {
    m_yaw = v;
    return *this;
}
float Frame::yaw() const noexcept {
    return m_yaw;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace sim {

int32_t KinematicState::ID() {
    return 1002;
}

const std::string KinematicState::ShortName() {
    return "KinematicState";
}
const std::string KinematicState::LongName() {
    return "opendlv.sim.KinematicState";
}

KinematicState& KinematicState::vx(const float &v) noexcept {
    m_vx = v;
    return *this;
}
float KinematicState::vx() const noexcept {
    return m_vx;
}

KinematicState& KinematicState::vy(const float &v) noexcept {
    m_vy = v;
    return *this;
}
float KinematicState::vy() const noexcept {
    return m_vy;
}

KinematicState& KinematicState::vz(const float &v) noexcept {
    m_vz = v;
    return *this;
}
float KinematicState::vz() const noexcept {
    return m_vz;
}

KinematicState& KinematicState::rollRate(const float &v) noexcept {
    m_rollRate = v;
    return *this;
}
float KinematicState::rollRate() const noexcept {
    return m_rollRate;
}

KinematicState& KinematicState::pitchRate(const float &v) noexcept {
    m_pitchRate = v;
    return *this;
}
float KinematicState::pitchRate() const noexcept {
    return m_pitchRate;
}

KinematicState& KinematicState::yawRate(const float &v) noexcept {
    m_yawRate = v;
    return *this;
}
float KinematicState::yawRate() const noexcept {
    return m_yawRate;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace body {

int32_t ComponentInfo::ID() {
    return 1021;
}

const std::string ComponentInfo::ShortName() {
    return "ComponentInfo";
}
const std::string ComponentInfo::LongName() {
    return "opendlv.body.ComponentInfo";
}

ComponentInfo& ComponentInfo::description(const std::string &v) noexcept {
    m_description = v;
    return *this;
}
std::string ComponentInfo::description() const noexcept {
    return m_description;
}

ComponentInfo& ComponentInfo::x(const float &v) noexcept {
    m_x = v;
    return *this;
}
float ComponentInfo::x() const noexcept {
    return m_x;
}

ComponentInfo& ComponentInfo::y(const float &v) noexcept {
    m_y = v;
    return *this;
}
float ComponentInfo::y() const noexcept {
    return m_y;
}

ComponentInfo& ComponentInfo::z(const float &v) noexcept {
    m_z = v;
    return *this;
}
float ComponentInfo::z() const noexcept {
    return m_z;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace body {

int32_t ActuatorInfo::ID() {
    return 1022;
}

const std::string ActuatorInfo::ShortName() {
    return "ActuatorInfo";
}
const std::string ActuatorInfo::LongName() {
    return "opendlv.body.ActuatorInfo";
}

ActuatorInfo& ActuatorInfo::description(const std::string &v) noexcept {
    m_description = v;
    return *this;
}
std::string ActuatorInfo::description() const noexcept {
    return m_description;
}

ActuatorInfo& ActuatorInfo::x(const float &v) noexcept {
    m_x = v;
    return *this;
}
float ActuatorInfo::x() const noexcept {
    return m_x;
}

ActuatorInfo& ActuatorInfo::y(const float &v) noexcept {
    m_y = v;
    return *this;
}
float ActuatorInfo::y() const noexcept {
    return m_y;
}

ActuatorInfo& ActuatorInfo::z(const float &v) noexcept {
    m_z = v;
    return *this;
}
float ActuatorInfo::z() const noexcept {
    return m_z;
}

ActuatorInfo& ActuatorInfo::signalId(const uint32_t &v) noexcept {
    m_signalId = v;
    return *this;
}
uint32_t ActuatorInfo::signalId() const noexcept {
    return m_signalId;
}

ActuatorInfo& ActuatorInfo::minValue(const float &v) noexcept {
    m_minValue = v;
    return *this;
}
float ActuatorInfo::minValue() const noexcept {
    return m_minValue;
}

ActuatorInfo& ActuatorInfo::maxValue(const float &v) noexcept {
    m_maxValue = v;
    return *this;
}
float ActuatorInfo::maxValue() const noexcept {
    return m_maxValue;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace body {

int32_t SensorInfo::ID() {
    return 1023;
}

const std::string SensorInfo::ShortName() {
    return "SensorInfo";
}
const std::string SensorInfo::LongName() {
    return "opendlv.body.SensorInfo";
}

SensorInfo& SensorInfo::description(const std::string &v) noexcept {
    m_description = v;
    return *this;
}
std::string SensorInfo::description() const noexcept {
    return m_description;
}

SensorInfo& SensorInfo::x(const float &v) noexcept {
    m_x = v;
    return *this;
}
float SensorInfo::x() const noexcept {
    return m_x;
}

SensorInfo& SensorInfo::y(const float &v) noexcept {
    m_y = v;
    return *this;
}
float SensorInfo::y() const noexcept {
    return m_y;
}

SensorInfo& SensorInfo::z(const float &v) noexcept {
    m_z = v;
    return *this;
}
float SensorInfo::z() const noexcept {
    return m_z;
}

SensorInfo& SensorInfo::signalId(const uint32_t &v) noexcept {
    m_signalId = v;
    return *this;
}
uint32_t SensorInfo::signalId() const noexcept {
    return m_signalId;
}

SensorInfo& SensorInfo::accuracyStd(const float &v) noexcept {
    m_accuracyStd = v;
    return *this;
}
float SensorInfo::accuracyStd() const noexcept {
    return m_accuracyStd;
}

SensorInfo& SensorInfo::minFrequency(const uint16_t &v) noexcept {
    m_minFrequency = v;
    return *this;
}
uint16_t SensorInfo::minFrequency() const noexcept {
    return m_minFrequency;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace body {

int32_t SignalInfo::ID() {
    return 1024;
}

const std::string SignalInfo::ShortName() {
    return "SignalInfo";
}
const std::string SignalInfo::LongName() {
    return "opendlv.body.SignalInfo";
}

SignalInfo& SignalInfo::description(const std::string &v) noexcept {
    m_description = v;
    return *this;
}
std::string SignalInfo::description() const noexcept {
    return m_description;
}

SignalInfo& SignalInfo::signalId(const uint32_t &v) noexcept {
    m_signalId = v;
    return *this;
}
uint32_t SignalInfo::signalId() const noexcept {
    return m_signalId;
}

SignalInfo& SignalInfo::accuracyStd(const float &v) noexcept {
    m_accuracyStd = v;
    return *this;
}
float SignalInfo::accuracyStd() const noexcept {
    return m_accuracyStd;
}

SignalInfo& SignalInfo::minFrequency(const uint16_t &v) noexcept {
    m_minFrequency = v;
    return *this;
}
uint16_t SignalInfo::minFrequency() const noexcept {
    return m_minFrequency;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t AccelerationReading::ID() {
    return 1030;
}

const std::string AccelerationReading::ShortName() {
    return "AccelerationReading";
}
const std::string AccelerationReading::LongName() {
    return "opendlv.proxy.AccelerationReading";
}

AccelerationReading& AccelerationReading::accelerationX(const float &v) noexcept {
    m_accelerationX = v;
    return *this;
}
float AccelerationReading::accelerationX() const noexcept {
    return m_accelerationX;
}

AccelerationReading& AccelerationReading::accelerationY(const float &v) noexcept {
    m_accelerationY = v;
    return *this;
}
float AccelerationReading::accelerationY() const noexcept {
    return m_accelerationY;
}

AccelerationReading& AccelerationReading::accelerationZ(const float &v) noexcept {
    m_accelerationZ = v;
    return *this;
}
float AccelerationReading::accelerationZ() const noexcept {
    return m_accelerationZ;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t AngularVelocityReading::ID() {
    return 1031;
}

const std::string AngularVelocityReading::ShortName() {
    return "AngularVelocityReading";
}
const std::string AngularVelocityReading::LongName() {
    return "opendlv.proxy.AngularVelocityReading";
}

AngularVelocityReading& AngularVelocityReading::angularVelocityX(const float &v) noexcept {
    m_angularVelocityX = v;
    return *this;
}
float AngularVelocityReading::angularVelocityX() const noexcept {
    return m_angularVelocityX;
}

AngularVelocityReading& AngularVelocityReading::angularVelocityY(const float &v) noexcept {
    m_angularVelocityY = v;
    return *this;
}
float AngularVelocityReading::angularVelocityY() const noexcept {
    return m_angularVelocityY;
}

AngularVelocityReading& AngularVelocityReading::angularVelocityZ(const float &v) noexcept {
    m_angularVelocityZ = v;
    return *this;
}
float AngularVelocityReading::angularVelocityZ() const noexcept {
    return m_angularVelocityZ;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t MagneticFieldReading::ID() {
    return 1032;
}

const std::string MagneticFieldReading::ShortName() {
    return "MagneticFieldReading";
}
const std::string MagneticFieldReading::LongName() {
    return "opendlv.proxy.MagneticFieldReading";
}

MagneticFieldReading& MagneticFieldReading::magneticFieldX(const float &v) noexcept {
    m_magneticFieldX = v;
    return *this;
}
float MagneticFieldReading::magneticFieldX() const noexcept {
    return m_magneticFieldX;
}

MagneticFieldReading& MagneticFieldReading::magneticFieldY(const float &v) noexcept {
    m_magneticFieldY = v;
    return *this;
}
float MagneticFieldReading::magneticFieldY() const noexcept {
    return m_magneticFieldY;
}

MagneticFieldReading& MagneticFieldReading::magneticFieldZ(const float &v) noexcept {
    m_magneticFieldZ = v;
    return *this;
}
float MagneticFieldReading::magneticFieldZ() const noexcept {
    return m_magneticFieldZ;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t AltitudeReading::ID() {
    return 1033;
}

const std::string AltitudeReading::ShortName() {
    return "AltitudeReading";
}
const std::string AltitudeReading::LongName() {
    return "opendlv.proxy.AltitudeReading";
}

AltitudeReading& AltitudeReading::altitude(const float &v) noexcept {
    m_altitude = v;
    return *this;
}
float AltitudeReading::altitude() const noexcept {
    return m_altitude;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t PressureReading::ID() {
    return 1034;
}

const std::string PressureReading::ShortName() {
    return "PressureReading";
}
const std::string PressureReading::LongName() {
    return "opendlv.proxy.PressureReading";
}

PressureReading& PressureReading::pressure(const float &v) noexcept {
    m_pressure = v;
    return *this;
}
float PressureReading::pressure() const noexcept {
    return m_pressure;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t TemperatureReading::ID() {
    return 1035;
}

const std::string TemperatureReading::ShortName() {
    return "TemperatureReading";
}
const std::string TemperatureReading::LongName() {
    return "opendlv.proxy.TemperatureReading";
}

TemperatureReading& TemperatureReading::temperature(const float &v) noexcept {
    m_temperature = v;
    return *this;
}
float TemperatureReading::temperature() const noexcept {
    return m_temperature;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t TorqueReading::ID() {
    return 1036;
}

const std::string TorqueReading::ShortName() {
    return "TorqueReading";
}
const std::string TorqueReading::LongName() {
    return "opendlv.proxy.TorqueReading";
}

TorqueReading& TorqueReading::torque(const float &v) noexcept {
    m_torque = v;
    return *this;
}
float TorqueReading::torque() const noexcept {
    return m_torque;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t VoltageReading::ID() {
    return 1037;
}

const std::string VoltageReading::ShortName() {
    return "VoltageReading";
}
const std::string VoltageReading::LongName() {
    return "opendlv.proxy.VoltageReading";
}

VoltageReading& VoltageReading::torque(const float &v) noexcept {
    m_torque = v;
    return *this;
}
float VoltageReading::torque() const noexcept {
    return m_torque;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t AngleReading::ID() {
    return 1038;
}

const std::string AngleReading::ShortName() {
    return "AngleReading";
}
const std::string AngleReading::LongName() {
    return "opendlv.proxy.AngleReading";
}

AngleReading& AngleReading::angle(const float &v) noexcept {
    m_angle = v;
    return *this;
}
float AngleReading::angle() const noexcept {
    return m_angle;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t SwitchStateReading::ID() {
    return 1040;
}

const std::string SwitchStateReading::ShortName() {
    return "SwitchStateReading";
}
const std::string SwitchStateReading::LongName() {
    return "opendlv.proxy.SwitchStateReading";
}

SwitchStateReading& SwitchStateReading::state(const int16_t &v) noexcept {
    m_state = v;
    return *this;
}
int16_t SwitchStateReading::state() const noexcept {
    return m_state;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t PedalPositionReading::ID() {
    return 1041;
}

const std::string PedalPositionReading::ShortName() {
    return "PedalPositionReading";
}
const std::string PedalPositionReading::LongName() {
    return "opendlv.proxy.PedalPositionReading";
}

PedalPositionReading& PedalPositionReading::position(const float &v) noexcept {
    m_position = v;
    return *this;
}
float PedalPositionReading::position() const noexcept {
    return m_position;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t GroundSteeringReading::ID() {
    return 1045;
}

const std::string GroundSteeringReading::ShortName() {
    return "GroundSteeringReading";
}
const std::string GroundSteeringReading::LongName() {
    return "opendlv.proxy.GroundSteeringReading";
}

GroundSteeringReading& GroundSteeringReading::groundSteering(const float &v) noexcept {
    m_groundSteering = v;
    return *this;
}
float GroundSteeringReading::groundSteering() const noexcept {
    return m_groundSteering;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t GroundSpeedReading::ID() {
    return 1046;
}

const std::string GroundSpeedReading::ShortName() {
    return "GroundSpeedReading";
}
const std::string GroundSpeedReading::LongName() {
    return "opendlv.proxy.GroundSpeedReading";
}

GroundSpeedReading& GroundSpeedReading::groundSpeed(const float &v) noexcept {
    m_groundSpeed = v;
    return *this;
}
float GroundSpeedReading::groundSpeed() const noexcept {
    return m_groundSpeed;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t WeightReading::ID() {
    return 1050;
}

const std::string WeightReading::ShortName() {
    return "WeightReading";
}
const std::string WeightReading::LongName() {
    return "opendlv.proxy.WeightReading";
}

WeightReading& WeightReading::torque(const float &v) noexcept {
    m_torque = v;
    return *this;
}
float WeightReading::torque() const noexcept {
    return m_torque;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t GeodeticHeadingReading::ID() {
    return 1051;
}

const std::string GeodeticHeadingReading::ShortName() {
    return "GeodeticHeadingReading";
}
const std::string GeodeticHeadingReading::LongName() {
    return "opendlv.proxy.GeodeticHeadingReading";
}

GeodeticHeadingReading& GeodeticHeadingReading::northHeading(const float &v) noexcept {
    m_northHeading = v;
    return *this;
}
float GeodeticHeadingReading::northHeading() const noexcept {
    return m_northHeading;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t GeodeticWgs84Reading::ID() {
    return 19;
}

const std::string GeodeticWgs84Reading::ShortName() {
    return "GeodeticWgs84Reading";
}
const std::string GeodeticWgs84Reading::LongName() {
    return "opendlv.proxy.GeodeticWgs84Reading";
}

GeodeticWgs84Reading& GeodeticWgs84Reading::latitude(const double &v) noexcept {
    m_latitude = v;
    return *this;
}
double GeodeticWgs84Reading::latitude() const noexcept {
    return m_latitude;
}

GeodeticWgs84Reading& GeodeticWgs84Reading::longitude(const double &v) noexcept {
    m_longitude = v;
    return *this;
}
double GeodeticWgs84Reading::longitude() const noexcept {
    return m_longitude;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t ImageReadingShared::ID() {
    return 14;
}

const std::string ImageReadingShared::ShortName() {
    return "ImageReadingShared";
}
const std::string ImageReadingShared::LongName() {
    return "opendlv.proxy.ImageReadingShared";
}

ImageReadingShared& ImageReadingShared::name(const std::string &v) noexcept {
    m_name = v;
    return *this;
}
std::string ImageReadingShared::name() const noexcept {
    return m_name;
}

ImageReadingShared& ImageReadingShared::size(const uint32_t &v) noexcept {
    m_size = v;
    return *this;
}
uint32_t ImageReadingShared::size() const noexcept {
    return m_size;
}

ImageReadingShared& ImageReadingShared::width(const uint32_t &v) noexcept {
    m_width = v;
    return *this;
}
uint32_t ImageReadingShared::width() const noexcept {
    return m_width;
}

ImageReadingShared& ImageReadingShared::height(const uint32_t &v) noexcept {
    m_height = v;
    return *this;
}
uint32_t ImageReadingShared::height() const noexcept {
    return m_height;
}

ImageReadingShared& ImageReadingShared::bytesPerPixel(const uint32_t &v) noexcept {
    m_bytesPerPixel = v;
    return *this;
}
uint32_t ImageReadingShared::bytesPerPixel() const noexcept {
    return m_bytesPerPixel;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t PointCloudReading::ID() {
    return 49;
}

const std::string PointCloudReading::ShortName() {
    return "PointCloudReading";
}
const std::string PointCloudReading::LongName() {
    return "opendlv.proxy.PointCloudReading";
}

PointCloudReading& PointCloudReading::startAzimuth(const float &v) noexcept {
    m_startAzimuth = v;
    return *this;
}
float PointCloudReading::startAzimuth() const noexcept {
    return m_startAzimuth;
}

PointCloudReading& PointCloudReading::endAzimuth(const float &v) noexcept {
    m_endAzimuth = v;
    return *this;
}
float PointCloudReading::endAzimuth() const noexcept {
    return m_endAzimuth;
}

PointCloudReading& PointCloudReading::entriesPerAzimuth(const uint8_t &v) noexcept {
    m_entriesPerAzimuth = v;
    return *this;
}
uint8_t PointCloudReading::entriesPerAzimuth() const noexcept {
    return m_entriesPerAzimuth;
}

PointCloudReading& PointCloudReading::distances(const std::string &v) noexcept {
    m_distances = v;
    return *this;
}
std::string PointCloudReading::distances() const noexcept {
    return m_distances;
}

PointCloudReading& PointCloudReading::numberOfBitsForIntensity(const uint8_t &v) noexcept {
    m_numberOfBitsForIntensity = v;
    return *this;
}
uint8_t PointCloudReading::numberOfBitsForIntensity() const noexcept {
    return m_numberOfBitsForIntensity;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t PointCloudReadingShared::ID() {
    return 28;
}

const std::string PointCloudReadingShared::ShortName() {
    return "PointCloudReadingShared";
}
const std::string PointCloudReadingShared::LongName() {
    return "opendlv.proxy.PointCloudReadingShared";
}

PointCloudReadingShared& PointCloudReadingShared::name(const std::string &v) noexcept {
    m_name = v;
    return *this;
}
std::string PointCloudReadingShared::name() const noexcept {
    return m_name;
}

PointCloudReadingShared& PointCloudReadingShared::size(const uint32_t &v) noexcept {
    m_size = v;
    return *this;
}
uint32_t PointCloudReadingShared::size() const noexcept {
    return m_size;
}

PointCloudReadingShared& PointCloudReadingShared::width(const uint32_t &v) noexcept {
    m_width = v;
    return *this;
}
uint32_t PointCloudReadingShared::width() const noexcept {
    return m_width;
}

PointCloudReadingShared& PointCloudReadingShared::height(const uint32_t &v) noexcept {
    m_height = v;
    return *this;
}
uint32_t PointCloudReadingShared::height() const noexcept {
    return m_height;
}

PointCloudReadingShared& PointCloudReadingShared::numberOfComponentsPerPoint(const uint8_t &v) noexcept {
    m_numberOfComponentsPerPoint = v;
    return *this;
}
uint8_t PointCloudReadingShared::numberOfComponentsPerPoint() const noexcept {
    return m_numberOfComponentsPerPoint;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t PressureRequest::ID() {
    return 1080;
}

const std::string PressureRequest::ShortName() {
    return "PressureRequest";
}
const std::string PressureRequest::LongName() {
    return "opendlv.proxy.PressureRequest";
}

PressureRequest& PressureRequest::pressure(const float &v) noexcept {
    m_pressure = v;
    return *this;
}
float PressureRequest::pressure() const noexcept {
    return m_pressure;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t TemperatureRequest::ID() {
    return 1081;
}

const std::string TemperatureRequest::ShortName() {
    return "TemperatureRequest";
}
const std::string TemperatureRequest::LongName() {
    return "opendlv.proxy.TemperatureRequest";
}

TemperatureRequest& TemperatureRequest::temperature(const float &v) noexcept {
    m_temperature = v;
    return *this;
}
float TemperatureRequest::temperature() const noexcept {
    return m_temperature;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t TorqueRequest::ID() {
    return 1082;
}

const std::string TorqueRequest::ShortName() {
    return "TorqueRequest";
}
const std::string TorqueRequest::LongName() {
    return "opendlv.proxy.TorqueRequest";
}

TorqueRequest& TorqueRequest::torque(const float &v) noexcept {
    m_torque = v;
    return *this;
}
float TorqueRequest::torque() const noexcept {
    return m_torque;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t VoltageRequest::ID() {
    return 1083;
}

const std::string VoltageRequest::ShortName() {
    return "VoltageRequest";
}
const std::string VoltageRequest::LongName() {
    return "opendlv.proxy.VoltageRequest";
}

VoltageRequest& VoltageRequest::torque(const float &v) noexcept {
    m_torque = v;
    return *this;
}
float VoltageRequest::torque() const noexcept {
    return m_torque;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t AngleRequest::ID() {
    return 1084;
}

const std::string AngleRequest::ShortName() {
    return "AngleRequest";
}
const std::string AngleRequest::LongName() {
    return "opendlv.proxy.AngleRequest";
}

AngleRequest& AngleRequest::angle(const float &v) noexcept {
    m_angle = v;
    return *this;
}
float AngleRequest::angle() const noexcept {
    return m_angle;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t SwitchStateRequest::ID() {
    return 1085;
}

const std::string SwitchStateRequest::ShortName() {
    return "SwitchStateRequest";
}
const std::string SwitchStateRequest::LongName() {
    return "opendlv.proxy.SwitchStateRequest";
}

SwitchStateRequest& SwitchStateRequest::state(const int16_t &v) noexcept {
    m_state = v;
    return *this;
}
int16_t SwitchStateRequest::state() const noexcept {
    return m_state;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t PedalPositionRequest::ID() {
    return 1086;
}

const std::string PedalPositionRequest::ShortName() {
    return "PedalPositionRequest";
}
const std::string PedalPositionRequest::LongName() {
    return "opendlv.proxy.PedalPositionRequest";
}

PedalPositionRequest& PedalPositionRequest::position(const float &v) noexcept {
    m_position = v;
    return *this;
}
float PedalPositionRequest::position() const noexcept {
    return m_position;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t PulseWidthModulationRequest::ID() {
    return 1087;
}

const std::string PulseWidthModulationRequest::ShortName() {
    return "PulseWidthModulationRequest";
}
const std::string PulseWidthModulationRequest::LongName() {
    return "opendlv.proxy.PulseWidthModulationRequest";
}

PulseWidthModulationRequest& PulseWidthModulationRequest::dutyCycleNs(const uint32_t &v) noexcept {
    m_dutyCycleNs = v;
    return *this;
}
uint32_t PulseWidthModulationRequest::dutyCycleNs() const noexcept {
    return m_dutyCycleNs;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t GroundSteeringRequest::ID() {
    return 1090;
}

const std::string GroundSteeringRequest::ShortName() {
    return "GroundSteeringRequest";
}
const std::string GroundSteeringRequest::LongName() {
    return "opendlv.proxy.GroundSteeringRequest";
}

GroundSteeringRequest& GroundSteeringRequest::groundSteering(const float &v) noexcept {
    m_groundSteering = v;
    return *this;
}
float GroundSteeringRequest::groundSteering() const noexcept {
    return m_groundSteering;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t GroundSpeedRequest::ID() {
    return 1091;
}

const std::string GroundSpeedRequest::ShortName() {
    return "GroundSpeedRequest";
}
const std::string GroundSpeedRequest::LongName() {
    return "opendlv.proxy.GroundSpeedRequest";
}

GroundSpeedRequest& GroundSpeedRequest::groundSpeed(const float &v) noexcept {
    m_groundSpeed = v;
    return *this;
}
float GroundSpeedRequest::groundSpeed() const noexcept {
    return m_groundSpeed;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t GroundAccelerationRequest::ID() {
    return 1092;
}

const std::string GroundAccelerationRequest::ShortName() {
    return "GroundAccelerationRequest";
}
const std::string GroundAccelerationRequest::LongName() {
    return "opendlv.proxy.GroundAccelerationRequest";
}

GroundAccelerationRequest& GroundAccelerationRequest::groundAcceleration(const float &v) noexcept {
    m_groundAcceleration = v;
    return *this;
}
float GroundAccelerationRequest::groundAcceleration() const noexcept {
    return m_groundAcceleration;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace proxy {

int32_t GroundDecelerationRequest::ID() {
    return 1093;
}

const std::string GroundDecelerationRequest::ShortName() {
    return "GroundDecelerationRequest";
}
const std::string GroundDecelerationRequest::LongName() {
    return "opendlv.proxy.GroundDecelerationRequest";
}

GroundDecelerationRequest& GroundDecelerationRequest::groundDeceleration(const float &v) noexcept {
    m_groundDeceleration = v;
    return *this;
}
float GroundDecelerationRequest::groundDeceleration() const noexcept {
    return m_groundDeceleration;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace system {

int32_t SignalStatusMessage::ID() {
    return 1100;
}

const std::string SignalStatusMessage::ShortName() {
    return "SignalStatusMessage";
}
const std::string SignalStatusMessage::LongName() {
    return "opendlv.system.SignalStatusMessage";
}

SignalStatusMessage& SignalStatusMessage::code(const int32_t &v) noexcept {
    m_code = v;
    return *this;
}
int32_t SignalStatusMessage::code() const noexcept {
    return m_code;
}

SignalStatusMessage& SignalStatusMessage::description(const std::string &v) noexcept {
    m_description = v;
    return *this;
}
std::string SignalStatusMessage::description() const noexcept {
    return m_description;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace system {

int32_t SystemOperationState::ID() {
    return 1101;
}

const std::string SystemOperationState::ShortName() {
    return "SystemOperationState";
}
const std::string SystemOperationState::LongName() {
    return "opendlv.system.SystemOperationState";
}

SystemOperationState& SystemOperationState::code(const int32_t &v) noexcept {
    m_code = v;
    return *this;
}
int32_t SystemOperationState::code() const noexcept {
    return m_code;
}

SystemOperationState& SystemOperationState::description(const std::string &v) noexcept {
    m_description = v;
    return *this;
}
std::string SystemOperationState::description() const noexcept {
    return m_description;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace system {

int32_t NetworkStatusMessage::ID() {
    return 1102;
}

const std::string NetworkStatusMessage::ShortName() {
    return "NetworkStatusMessage";
}
const std::string NetworkStatusMessage::LongName() {
    return "opendlv.system.NetworkStatusMessage";
}

NetworkStatusMessage& NetworkStatusMessage::code(const int32_t &v) noexcept {
    m_code = v;
    return *this;
}
int32_t NetworkStatusMessage::code() const noexcept {
    return m_code;
}

NetworkStatusMessage& NetworkStatusMessage::description(const std::string &v) noexcept {
    m_description = v;
    return *this;
}
std::string NetworkStatusMessage::description() const noexcept {
    return m_description;
}

}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace logic { namespace sensation {

int32_t Direction::ID() {
    return 1110;
}

const std::string Direction::ShortName() {
    return "Direction";
}
const std::string Direction::LongName() {
    return "opendlv.logic.sensation.Direction";
}

Direction& Direction::azimuthAngle(const float &v) noexcept {
    m_azimuthAngle = v;
    return *this;
}
float Direction::azimuthAngle() const noexcept {
    return m_azimuthAngle;
}

Direction& Direction::zenithAngle(const float &v) noexcept {
    m_zenithAngle = v;
    return *this;
}
float Direction::zenithAngle() const noexcept {
    return m_zenithAngle;
}

}}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace logic { namespace sensation {

int32_t Point::ID() {
    return 1111;
}

const std::string Point::ShortName() {
    return "Point";
}
const std::string Point::LongName() {
    return "opendlv.logic.sensation.Point";
}

Point& Point::azimuthAngle(const float &v) noexcept {
    m_azimuthAngle = v;
    return *this;
}
float Point::azimuthAngle() const noexcept {
    return m_azimuthAngle;
}

Point& Point::zenithAngle(const float &v) noexcept {
    m_zenithAngle = v;
    return *this;
}
float Point::zenithAngle() const noexcept {
    return m_zenithAngle;
}

Point& Point::distance(const float &v) noexcept {
    m_distance = v;
    return *this;
}
float Point::distance() const noexcept {
    return m_distance;
}

}}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace logic { namespace sensation {

int32_t Geolocation::ID() {
    return 1116;
}

const std::string Geolocation::ShortName() {
    return "Geolocation";
}
const std::string Geolocation::LongName() {
    return "opendlv.logic.sensation.Geolocation";
}

Geolocation& Geolocation::latitude(const float &v) noexcept {
    m_latitude = v;
    return *this;
}
float Geolocation::latitude() const noexcept {
    return m_latitude;
}

Geolocation& Geolocation::longitude(const float &v) noexcept {
    m_longitude = v;
    return *this;
}
float Geolocation::longitude() const noexcept {
    return m_longitude;
}

Geolocation& Geolocation::altitude(const float &v) noexcept {
    m_altitude = v;
    return *this;
}
float Geolocation::altitude() const noexcept {
    return m_altitude;
}

Geolocation& Geolocation::heading(const float &v) noexcept {
    m_heading = v;
    return *this;
}
float Geolocation::heading() const noexcept {
    return m_heading;
}

}}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace logic { namespace sensation {

int32_t Equilibrioception::ID() {
    return 1017;
}

const std::string Equilibrioception::ShortName() {
    return "Equilibrioception";
}
const std::string Equilibrioception::LongName() {
    return "opendlv.logic.sensation.Equilibrioception";
}

Equilibrioception& Equilibrioception::vx(const float &v) noexcept {
    m_vx = v;
    return *this;
}
float Equilibrioception::vx() const noexcept {
    return m_vx;
}

Equilibrioception& Equilibrioception::vy(const float &v) noexcept {
    m_vy = v;
    return *this;
}
float Equilibrioception::vy() const noexcept {
    return m_vy;
}

Equilibrioception& Equilibrioception::vz(const float &v) noexcept {
    m_vz = v;
    return *this;
}
float Equilibrioception::vz() const noexcept {
    return m_vz;
}

Equilibrioception& Equilibrioception::rollRate(const float &v) noexcept {
    m_rollRate = v;
    return *this;
}
float Equilibrioception::rollRate() const noexcept {
    return m_rollRate;
}

Equilibrioception& Equilibrioception::pitchRate(const float &v) noexcept {
    m_pitchRate = v;
    return *this;
}
float Equilibrioception::pitchRate() const noexcept {
    return m_pitchRate;
}

Equilibrioception& Equilibrioception::yawRate(const float &v) noexcept {
    m_yawRate = v;
    return *this;
}
float Equilibrioception::yawRate() const noexcept {
    return m_yawRate;
}

}}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace logic { namespace perception {

int32_t Object::ID() {
    return 1130;
}

const std::string Object::ShortName() {
    return "Object";
}
const std::string Object::LongName() {
    return "opendlv.logic.perception.Object";
}

Object& Object::objectId(const uint32_t &v) noexcept {
    m_objectId = v;
    return *this;
}
uint32_t Object::objectId() const noexcept {
    return m_objectId;
}

}}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace logic { namespace perception {

int32_t ObjectType::ID() {
    return 1131;
}

const std::string ObjectType::ShortName() {
    return "ObjectType";
}
const std::string ObjectType::LongName() {
    return "opendlv.logic.perception.ObjectType";
}

ObjectType& ObjectType::objectId(const uint32_t &v) noexcept {
    m_objectId = v;
    return *this;
}
uint32_t ObjectType::objectId() const noexcept {
    return m_objectId;
}

ObjectType& ObjectType::type(const uint32_t &v) noexcept {
    m_type = v;
    return *this;
}
uint32_t ObjectType::type() const noexcept {
    return m_type;
}

}}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace logic { namespace perception {

int32_t ObjectProperty::ID() {
    return 1132;
}

const std::string ObjectProperty::ShortName() {
    return "ObjectProperty";
}
const std::string ObjectProperty::LongName() {
    return "opendlv.logic.perception.ObjectProperty";
}

ObjectProperty& ObjectProperty::objectId(const uint32_t &v) noexcept {
    m_objectId = v;
    return *this;
}
uint32_t ObjectProperty::objectId() const noexcept {
    return m_objectId;
}

ObjectProperty& ObjectProperty::property(const std::string &v) noexcept {
    m_property = v;
    return *this;
}
std::string ObjectProperty::property() const noexcept {
    return m_property;
}

}}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace logic { namespace perception {

int32_t ObjectDirection::ID() {
    return 1133;
}

const std::string ObjectDirection::ShortName() {
    return "ObjectDirection";
}
const std::string ObjectDirection::LongName() {
    return "opendlv.logic.perception.ObjectDirection";
}

ObjectDirection& ObjectDirection::objectId(const uint32_t &v) noexcept {
    m_objectId = v;
    return *this;
}
uint32_t ObjectDirection::objectId() const noexcept {
    return m_objectId;
}

ObjectDirection& ObjectDirection::azimuthAngle(const float &v) noexcept {
    m_azimuthAngle = v;
    return *this;
}
float ObjectDirection::azimuthAngle() const noexcept {
    return m_azimuthAngle;
}

ObjectDirection& ObjectDirection::zenithAngle(const float &v) noexcept {
    m_zenithAngle = v;
    return *this;
}
float ObjectDirection::zenithAngle() const noexcept {
    return m_zenithAngle;
}

}}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace logic { namespace perception {

int32_t ObjectDistance::ID() {
    return 1134;
}

const std::string ObjectDistance::ShortName() {
    return "ObjectDistance";
}
const std::string ObjectDistance::LongName() {
    return "opendlv.logic.perception.ObjectDistance";
}

ObjectDistance& ObjectDistance::objectId(const uint32_t &v) noexcept {
    m_objectId = v;
    return *this;
}
uint32_t ObjectDistance::objectId() const noexcept {
    return m_objectId;
}

ObjectDistance& ObjectDistance::distance(const float &v) noexcept {
    m_distance = v;
    return *this;
}
float ObjectDistance::distance() const noexcept {
    return m_distance;
}

}}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace logic { namespace perception {

int32_t ObjectAngularBlob::ID() {
    return 1135;
}

const std::string ObjectAngularBlob::ShortName() {
    return "ObjectAngularBlob";
}
const std::string ObjectAngularBlob::LongName() {
    return "opendlv.logic.perception.ObjectAngularBlob";
}

ObjectAngularBlob& ObjectAngularBlob::objectId(const uint32_t &v) noexcept {
    m_objectId = v;
    return *this;
}
uint32_t ObjectAngularBlob::objectId() const noexcept {
    return m_objectId;
}

ObjectAngularBlob& ObjectAngularBlob::width(const float &v) noexcept {
    m_width = v;
    return *this;
}
float ObjectAngularBlob::width() const noexcept {
    return m_width;
}

ObjectAngularBlob& ObjectAngularBlob::height(const float &v) noexcept {
    m_height = v;
    return *this;
}
float ObjectAngularBlob::height() const noexcept {
    return m_height;
}

}}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace logic { namespace perception {

int32_t GroundSurface::ID() {
    return 1140;
}

const std::string GroundSurface::ShortName() {
    return "GroundSurface";
}
const std::string GroundSurface::LongName() {
    return "opendlv.logic.perception.GroundSurface";
}

GroundSurface& GroundSurface::surfaceId(const uint32_t &v) noexcept {
    m_surfaceId = v;
    return *this;
}
uint32_t GroundSurface::surfaceId() const noexcept {
    return m_surfaceId;
}

}}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace logic { namespace perception {

int32_t GroundSurfaceType::ID() {
    return 1141;
}

const std::string GroundSurfaceType::ShortName() {
    return "GroundSurfaceType";
}
const std::string GroundSurfaceType::LongName() {
    return "opendlv.logic.perception.GroundSurfaceType";
}

GroundSurfaceType& GroundSurfaceType::surfaceId(const uint32_t &v) noexcept {
    m_surfaceId = v;
    return *this;
}
uint32_t GroundSurfaceType::surfaceId() const noexcept {
    return m_surfaceId;
}

GroundSurfaceType& GroundSurfaceType::type(const uint32_t &v) noexcept {
    m_type = v;
    return *this;
}
uint32_t GroundSurfaceType::type() const noexcept {
    return m_type;
}

}}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace logic { namespace perception {

int32_t GroundSurfaceProperty::ID() {
    return 1142;
}

const std::string GroundSurfaceProperty::ShortName() {
    return "GroundSurfaceProperty";
}
const std::string GroundSurfaceProperty::LongName() {
    return "opendlv.logic.perception.GroundSurfaceProperty";
}

GroundSurfaceProperty& GroundSurfaceProperty::surfaceId(const uint32_t &v) noexcept {
    m_surfaceId = v;
    return *this;
}
uint32_t GroundSurfaceProperty::surfaceId() const noexcept {
    return m_surfaceId;
}

GroundSurfaceProperty& GroundSurfaceProperty::property(const std::string &v) noexcept {
    m_property = v;
    return *this;
}
std::string GroundSurfaceProperty::property() const noexcept {
    return m_property;
}

}}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace logic { namespace perception {

int32_t GroundSurfaceArea::ID() {
    return 1143;
}

const std::string GroundSurfaceArea::ShortName() {
    return "GroundSurfaceArea";
}
const std::string GroundSurfaceArea::LongName() {
    return "opendlv.logic.perception.GroundSurfaceArea";
}

GroundSurfaceArea& GroundSurfaceArea::surfaceId(const uint32_t &v) noexcept {
    m_surfaceId = v;
    return *this;
}
uint32_t GroundSurfaceArea::surfaceId() const noexcept {
    return m_surfaceId;
}

GroundSurfaceArea& GroundSurfaceArea::leftPerpendicularAngle(const float &v) noexcept {
    m_leftPerpendicularAngle = v;
    return *this;
}
float GroundSurfaceArea::leftPerpendicularAngle() const noexcept {
    return m_leftPerpendicularAngle;
}

GroundSurfaceArea& GroundSurfaceArea::leftDistance(const float &v) noexcept {
    m_leftDistance = v;
    return *this;
}
float GroundSurfaceArea::leftDistance() const noexcept {
    return m_leftDistance;
}

GroundSurfaceArea& GroundSurfaceArea::rightPerpendicularAngle(const float &v) noexcept {
    m_rightPerpendicularAngle = v;
    return *this;
}
float GroundSurfaceArea::rightPerpendicularAngle() const noexcept {
    return m_rightPerpendicularAngle;
}

GroundSurfaceArea& GroundSurfaceArea::rightDistance(const float &v) noexcept {
    m_rightDistance = v;
    return *this;
}
float GroundSurfaceArea::rightDistance() const noexcept {
    return m_rightDistance;
}

GroundSurfaceArea& GroundSurfaceArea::nearPerpendicularAngle(const float &v) noexcept {
    m_nearPerpendicularAngle = v;
    return *this;
}
float GroundSurfaceArea::nearPerpendicularAngle() const noexcept {
    return m_nearPerpendicularAngle;
}

GroundSurfaceArea& GroundSurfaceArea::nearDistance(const float &v) noexcept {
    m_nearDistance = v;
    return *this;
}
float GroundSurfaceArea::nearDistance() const noexcept {
    return m_nearDistance;
}

GroundSurfaceArea& GroundSurfaceArea::farPerpendicularAngle(const float &v) noexcept {
    m_farPerpendicularAngle = v;
    return *this;
}
float GroundSurfaceArea::farPerpendicularAngle() const noexcept {
    return m_farPerpendicularAngle;
}

GroundSurfaceArea& GroundSurfaceArea::farDistance(const float &v) noexcept {
    m_farDistance = v;
    return *this;
}
float GroundSurfaceArea::farDistance() const noexcept {
    return m_farDistance;
}

}}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace logic { namespace action {

int32_t AimDirection::ID() {
    return 1171;
}

const std::string AimDirection::ShortName() {
    return "AimDirection";
}
const std::string AimDirection::LongName() {
    return "opendlv.logic.action.AimDirection";
}

AimDirection& AimDirection::azimuthAngle(const float &v) noexcept {
    m_azimuthAngle = v;
    return *this;
}
float AimDirection::azimuthAngle() const noexcept {
    return m_azimuthAngle;
}

AimDirection& AimDirection::zentihAngle(const float &v) noexcept {
    m_zentihAngle = v;
    return *this;
}
float AimDirection::zentihAngle() const noexcept {
    return m_zentihAngle;
}

}}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace logic { namespace action {

int32_t AimPoint::ID() {
    return 1172;
}

const std::string AimPoint::ShortName() {
    return "AimPoint";
}
const std::string AimPoint::LongName() {
    return "opendlv.logic.action.AimPoint";
}

AimPoint& AimPoint::azimuthAngle(const float &v) noexcept {
    m_azimuthAngle = v;
    return *this;
}
float AimPoint::azimuthAngle() const noexcept {
    return m_azimuthAngle;
}

AimPoint& AimPoint::zenithAngle(const float &v) noexcept {
    m_zenithAngle = v;
    return *this;
}
float AimPoint::zenithAngle() const noexcept {
    return m_zenithAngle;
}

AimPoint& AimPoint::distance(const float &v) noexcept {
    m_distance = v;
    return *this;
}
float AimPoint::distance() const noexcept {
    return m_distance;
}

}}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace logic { namespace action {

int32_t PreviewPoint::ID() {
    return 1173;
}

const std::string PreviewPoint::ShortName() {
    return "PreviewPoint";
}
const std::string PreviewPoint::LongName() {
    return "opendlv.logic.action.PreviewPoint";
}

PreviewPoint& PreviewPoint::azimuthAngle(const float &v) noexcept {
    m_azimuthAngle = v;
    return *this;
}
float PreviewPoint::azimuthAngle() const noexcept {
    return m_azimuthAngle;
}

PreviewPoint& PreviewPoint::zenithAngle(const float &v) noexcept {
    m_zenithAngle = v;
    return *this;
}
float PreviewPoint::zenithAngle() const noexcept {
    return m_zenithAngle;
}

PreviewPoint& PreviewPoint::distance(const float &v) noexcept {
    m_distance = v;
    return *this;
}
float PreviewPoint::distance() const noexcept {
    return m_distance;
}

}}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace logic { namespace cognition {

int32_t GroundSteeringLimit::ID() {
    return 1191;
}

const std::string GroundSteeringLimit::ShortName() {
    return "GroundSteeringLimit";
}
const std::string GroundSteeringLimit::LongName() {
    return "opendlv.logic.cognition.GroundSteeringLimit";
}

GroundSteeringLimit& GroundSteeringLimit::steeringLimit(const float &v) noexcept {
    m_steeringLimit = v;
    return *this;
}
float GroundSteeringLimit::steeringLimit() const noexcept {
    return m_steeringLimit;
}

}}}

#include <opendlv-standard-message-set.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */
namespace opendlv { namespace logic { namespace cognition {

int32_t GroundSpeedLimit::ID() {
    return 1192;
}

const std::string GroundSpeedLimit::ShortName() {
    return "GroundSpeedLimit";
}
const std::string GroundSpeedLimit::LongName() {
    return "opendlv.logic.cognition.GroundSpeedLimit";
}

GroundSpeedLimit& GroundSpeedLimit::speedLimit(const float &v) noexcept {
    m_speedLimit = v;
    return *this;
}
float GroundSpeedLimit::speedLimit() const noexcept {
    return m_speedLimit;
}

}}}

