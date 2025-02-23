#pragma once
#include "okapi/impl/device/distanceSensor.hpp"
#include "16868C/devices/inertial.hpp"
#include "16868C/devices/trackingWheel.hpp"
#include "16868C/util/math.hpp"
#include "16868C/util/pose.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "pros/rtos.hpp"
#include <array>

using namespace okapi::literals;

namespace lib16868C {
struct DistanceSensor {
	okapi::DistanceSensor* snsr { nullptr };
	double offset = 0;

	DistanceSensor();
	DistanceSensor(okapi::DistanceSensor* snsr, okapi::QLength offset);

	double getDist() const;
	double getConfidence() const;
};

class Odometry {
	public:
		Odometry();
		Odometry(std::array<TrackingWheel, 3> trackingWheels, std::array<DistanceSensor, 4> distanceSensors, Inertial* inertial);
		Odometry(TrackingWheel left, TrackingWheel right, TrackingWheel middle);
		Odometry(TrackingWheel left, TrackingWheel right, TrackingWheel middle, Inertial* inertial);
		Odometry(DistanceSensor front, DistanceSensor right, DistanceSensor rear, DistanceSensor left, Inertial* inertial);
		Odometry(Odometry& odom);

		void calibrate();

		void init();
		void init(Pose pose);

		Pose getPose();
		Pose getState();

		Pose getVel();

		void update(bool front, bool right, bool rear, bool left);
		void update(okapi::QLength x, okapi::QLength y);
		void update(okapi::QLength x, okapi::QLength y, okapi::QAngle theta);
		void update(Pose pose);

		std::array<TrackingWheel*, 3> getEncoders() const;
		std::array<DistanceSensor*, 4> getDistanceSensors() const;
		Inertial* getInertial() const;
		
		void resetSensors();

	private:
		void updateVel(Pose vel);

		Pose pose { 0_in, 0_in, 0_deg, 0 };
		pros::Mutex poseMutex;
		Pose prevPose { 0_in, 0_in, 0_deg, 0 };

		Pose vel {};
		pros::Mutex velMutex;
		Pose prevVel {};

		okapi::QTime dt = 10_ms;

		TrackingWheel leftEnc;
		TrackingWheel rightEnc;
		TrackingWheel middleEnc;
		std::array<TrackingWheel*, 3> trackingWheels = { nullptr, nullptr, nullptr };

		DistanceSensor frontDist;
		DistanceSensor rightDist;
		DistanceSensor rearDist;
		DistanceSensor leftDist;
		std::array<DistanceSensor*, 4> distanceSensors = { nullptr, nullptr, nullptr, nullptr };

		Inertial* inertial { nullptr };

		pros::Task odomTask = pros::Task(odomManager, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odometry");
		static void odomManager(void* param);

		const double MAX_DELTA = (10_in).convert(okapi::inch);
		const double FIELD_WIDTH = (12_ft).convert(okapi::inch);
		const int MIN_CONFIDENCE = 9;

		Point p_00 { 0, 0 };
		Point p_10 { FIELD_WIDTH, 0 };
		Point p_01 { 0, FIELD_WIDTH };
		Point p_11 { FIELD_WIDTH, FIELD_WIDTH };
		LineSegment north {p_10, p_11};
		LineSegment south {p_00, p_01};
		LineSegment east {p_00, p_10};
		LineSegment west {p_01, p_11};
		std::array<LineSegment, 4> walls { east, north, west, south };

		void step(std::array<double, 4> deltas);
};
} // namespace lib16868C