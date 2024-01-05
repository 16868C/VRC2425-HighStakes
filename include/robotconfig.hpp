#pragma once

#include "okapi/api.hpp"
#include "16868C/subsystems/chassis/odometry.hpp"
#include "16868C/subsystems/chassis/inline.hpp"
#include "16868C/subsystems/intake.hpp"
#include "16868C/subsystems/catapult.hpp"
#include "16868C/devices/abstractEncoder.hpp"
#include "16868C/devices/pneumatic.hpp"
#include "16868C/devices/rotation.hpp"

using namespace okapi::literals;

// Controllers
extern okapi::Controller master;