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

	inline void reset() {
		if (left) left->resetZero();
		if (right) right->resetZero();
		if (middle) middle->resetZero();
		if (inertial) inertial->reset(true);
	}
	inline std::vector<double> getTicks() {
		if (right) return {left->get(), right->get(), middle->get()};
		else return {left->get(), 0, middle->get()};
	}
	inline std::vector<int> getTPR() {
		if (right) return {left->getTPR(), right->getTPR(), middle->getTPR()};
		else return {left->getTPR(), 0x3f, middle->getTPR()};
	}
};

struct EncoderScales {
	okapi::QLength leftDiam { 0_in };
	okapi::QLength rightDiam { 0_in };
	okapi::QLength wheelTrack { 0_in };
	okapi::QLength middleDiam { 0_in };
	okapi::QLength middleTrack { 0_in };

	EncoderScales(okapi::QLength leftDiam, okapi::QLength rightDiam, okapi::QLength wheelTrack, okapi::QLength middleDiam, okapi::QLength middleTrack)
				: leftDiam(leftDiam), rightDiam(rightDiam), wheelTrack(wheelTrack), middleDiam(middleDiam), middleTrack(middleTrack) {}
	EncoderScales(okapi::QLength leftDiam, okapi::QLength middleDiam, okapi::QLength middleTrack)
				: leftDiam(leftDiam), middleDiam(middleDiam), middleTrack(middleTrack) {}

	inline okapi::QLength getDiam(int index) {
		switch(index) {
			case 0: return leftDiam;
			case 1: return rightDiam;
			case 2: return middleDiam;
			default: return 0_in;
		}
	}
	inline okapi::QLength getTrack(int index) {
		switch(index) {
			case 0: return wheelTrack;
			case 1: return wheelTrack;
			case 2: return middleTrack;
			default: return 0_in;
		}
	}
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
		inline std::vector<double> ticksToDist(std::vector<double> ticks, std::vector<int> tpr) {
			if (ticks.size() != tpr.size()) { std::cerr << "ticksToDist: ticks and tpr must be the same size\n"; return std::vector<double>(ticks.size(), 0); }

			std::vector<double> dists;
			for (int i = 0; i < ticks.size(); i++)
				dists.push_back(ticks[i] / tpr[i] * (encScales.getDiam(i) * okapi::pi).convert(okapi::inch));
			return dists;
		}

		Pose pose { 0_in, 0_in, 0_deg, 0 };

		OdomSensors snsrs;
		EncoderScales encScales;

		pros::task_t odomTask;
		static void odomManager(void* param);

		const okapi::QLength MAX_DELTA = 10_in;

		void step(std::vector<double> deltas);
};
} // namespace lib16868C