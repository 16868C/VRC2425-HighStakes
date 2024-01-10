#pragma once
#include "16868C/devices/abstractEncoder.hpp"
#include "16868C/devices/inertial.hpp"
#include "16868C/devices/rotation.hpp"
#include "16868C/devices/opticalEncoder.hpp"
#include "16868C/util/pose.hpp"
#include "16868C/util/util.hpp"
#include "api.h"
#include <memory>

using namespace okapi::literals;

namespace lib16868C {
struct Encoders {
	std::shared_ptr<AbstractEncoder> left { nullptr };
	std::shared_ptr<AbstractEncoder> right { nullptr };
	std::shared_ptr<AbstractEncoder> middle { nullptr };

	std::vector<double> getTicks() const;
	std::vector<int> getTPR() const;

	void reset();
};
struct DistanceSnsrs {
	std::shared_ptr<okapi::DistanceSensor> front { nullptr };
	std::shared_ptr<okapi::DistanceSensor> back { nullptr };
	std::shared_ptr<okapi::DistanceSensor> left { nullptr };
	std::shared_ptr<okapi::DistanceSensor> right { nullptr };

	std::vector<double> getDists() const;
	std::vector<int> getConfidences() const;
};

struct EncoderScales {
	double leftDiam = 0;
	double leftGearRatio = 0;
	double rightDiam = 0;
	double rightGearRatio = 0;
	double wheelTrack = 0;
	double middleDiam = 0;
	double middleTrack = 0;

	EncoderScales();
	EncoderScales(okapi::QLength leftDiam, double leftGearRatio, okapi::QLength rightDiam, double rightGearRatio, okapi::QLength wheelTrack, okapi::QLength middleDiam, okapi::QLength middleTrack);
	EncoderScales(okapi::QLength leftDiam, double leftGearRatio, okapi::QLength middleDiam, okapi::QLength middleTrack);

	double getDiam(int index);
	double getTrack(int index);
};
struct DistanceScales {
	double frontDist = 0;
	double backDist = 0;
	double leftDist = 0;
	double rightDist = 0;

	double getDist(int index);
};

class Odometry {
	public:
		Odometry();
		Odometry(Encoders snsrs, DistanceSnsrs dists, std::shared_ptr<Inertial> inertial, EncoderScales encScales, DistanceScales distScales);
		Odometry(std::vector<std::shared_ptr<AbstractEncoder>> encs, EncoderScales encScales);
		Odometry(std::vector<std::shared_ptr<AbstractEncoder>> encs, Inertial inertial, EncoderScales encScales);
		Odometry(std::vector<std::shared_ptr<okapi::DistanceSensor>> dists, Inertial inertial, DistanceScales distScales);
		Odometry(Odometry& odom);

		void init();
		void init(Pose pose);

		Pose getPose();
		Pose getState();

		void update();
		void update(Pose pose);

		Encoders getEncoders() const;
		DistanceSnsrs getDistanceSensors() const;
		std::shared_ptr<Inertial> getInertial() const;
		EncoderScales getEncoderScales() const;
		DistanceScales getDistanceScales() const;
		
		void resetSensors();

	private:
		std::vector<double> ticksToDist(std::vector<double> ticks, std::vector<int> tpr);

		Pose pose { 0_in, 0_in, 0_deg, 0 };
		pros::Mutex poseMutex;
		Pose prevPose { 0_in, 0_in, 0_deg, 0 };

		Encoders encs;
		DistanceSnsrs dists;
		std::shared_ptr<Inertial> inertial { nullptr };
		EncoderScales encScales;
		DistanceScales distScales;

		pros::task_t odomTask;
		static void odomManager(void* param);

		const okapi::QLength MAX_DELTA = 10_in;
		const okapi::QLength FIELD_WIDTH = 12_ft;
		const int MIN_CONFIDENCE = 9;

		#define fieldWidth FIELD_WIDTH.convert(okapi::inch)
		Point p_00 { 0, 0 };
		Point p_10 { fieldWidth, 0 };
		Point p_01 { 0, fieldWidth };
		Point p_11 { fieldWidth, fieldWidth };
		LineSegment north {p_01, p_11};
		LineSegment south {p_00, p_10};
		LineSegment east {p_10, p_11};
		LineSegment west {p_00, p_01};

		void step(std::vector<double> deltas);
};
} // namespace lib16868C