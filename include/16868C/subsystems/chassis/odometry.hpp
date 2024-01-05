#pragma once
#include "16868C/devices/abstractEncoder.hpp"
#include "16868C/devices/rotation.hpp"
#include "16868C/devices/opticalEncoder.hpp"
#include "16868C/util/pose.hpp"
#include "16868C/util/util.hpp"
#include "api.h"
#include <memory>

using namespace okapi::literals;

namespace lib16868C {
struct OdomSensors {
	std::shared_ptr<AbstractEncoder> left { nullptr };
	std::shared_ptr<AbstractEncoder> right { nullptr };
	std::shared_ptr<AbstractEncoder> middle { nullptr };
	std::shared_ptr<pros::Imu> inertial { nullptr };

	void reset();
	std::vector<double> getTicks();
	std::vector<int> getTPR();
};

struct EncoderScales {
	double leftDiam;
	double rightDiam = 0;
	double wheelTrack = 0;
	double middleDiam;
	double middleTrack;

	EncoderScales(okapi::QLength leftDiam, okapi::QLength rightDiam, okapi::QLength wheelTrack, okapi::QLength middleDiam, okapi::QLength middleTrack);
	EncoderScales(okapi::QLength leftDiam, okapi::QLength middleDiam, okapi::QLength middleTrack);

	double getDiam(int index);
	double getTrack(int index);
};

class Odometry {
	public:
		Odometry(OdomSensors snsrs, EncoderScales encScales);
		Odometry(std::vector<Rotation> encs, EncoderScales encScales);
		Odometry(std::vector<Rotation> encs, pros::Imu inertial, EncoderScales encScales);
		Odometry(std::vector<OpticalEncoder> encs, EncoderScales encScales);
		Odometry(std::vector<OpticalEncoder> encs, pros::Imu inertial, EncoderScales encScales);

		void init();
		void init(Pose pose);

		Pose getPose();
		Pose getState();

		OdomSensors getSensors();
		void resetSensors();

		EncoderScales getEncoderScales();

	private:
		std::vector<double> ticksToDist(std::vector<double> ticks, std::vector<int> tpr);

		Pose pose { 0_in, 0_in, 0_deg, 0 };

		OdomSensors snsrs;
		EncoderScales encScales;

		pros::task_t odomTask;
		static void odomManager(void* param);

		const okapi::QLength MAX_DELTA = 10_in;

		void step(std::vector<double> deltas);
};
} // namespace lib16868C