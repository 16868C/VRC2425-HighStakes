#pragma once
#include "16868X/devices/abstractEncoder.hpp"
#include "16868X/util/pose.hpp"
#include "16868X/util/util.hpp"
#include "api.h"
#include <memory>

namespace lib16868X {
enum class EncoderValsType {
	DISTANCE,
	TICKS,
	SCALES_WHEEL_DIAM,
	SCALES_WHEEL_TRACK
};

struct EncoderVals {
	double left { 0 };
	double right { 0 };
	double rear { 0 };
	double theta { 0 };

	EncoderValsType type { EncoderValsType::DISTANCE };

	EncoderVals(EncoderValsType type) : type(type) {}
	EncoderVals(double left, double right, double rear, EncoderValsType type)
		: left(left), right(right), rear(rear), type(type) {}
	EncoderVals(double left, double right, double rear, double theta, EncoderValsType type) 
		: left(left), right(right), rear(rear), theta(theta), type(type) {}

	inline EncoderVals operator+(const EncoderVals& other) {
		if (type != other.type) return *this;
		return {left + other.left, right + other.right, rear + other.rear, theta, type};
	}
	inline EncoderVals operator-(const EncoderVals& other) {
		if (type != other.type) return *this;
		return {left - other.left, right - other.right, rear - other.rear, theta, type};
	}
	inline EncoderVals operator*(const EncoderVals& other) {
		if (type != other.type) return *this;
		return {left * other.left, right * other.right, rear * other.rear, theta, type};
	}
	inline EncoderVals operator/(const EncoderVals& other) {
		if (type != other.type) return *this;
		return {left / other.left, right / other.right, rear / other.rear, theta, type};
	}

	inline EncoderVals operator*(const double& factor) {
		return {left * factor, right * factor, rear * factor, theta, type};
	}
	inline EncoderVals operator/(const double& divisor) {
		return {left / divisor, right / divisor, rear / divisor, theta, type};
	}
};
typedef EncoderVals EncoderTicks;
typedef EncoderVals EncoderScales;

struct Encoders {
	std::shared_ptr<AbstractEncoder> left { nullptr };
	std::shared_ptr<AbstractEncoder> right { nullptr };
	std::shared_ptr<AbstractEncoder> rear { nullptr };

	int tpr = left->getTPR();

	inline EncoderTicks getTicks() {
		return {left->get(), right->get(), rear->get(), EncoderValsType::TICKS};
	}
	inline void reset() {
		left->resetZero();
		right->resetZero();
		rear->resetZero();
	}
};

class Odometry {
	public:
		std::shared_ptr<Odometry> getOdometry();
		std::shared_ptr<Odometry> getOdometry(Encoders encs, EncoderScales wheelDiamScales, EncoderScales wheelTrackScales);
		std::shared_ptr<Odometry> getOdometry(Encoders encs, EncoderScales wheelDiamScales, EncoderScales wheelTrackScales, std::shared_ptr<pros::Imu> inertial);

		void init();
		void init(Pose pose);

		Pose getPose();

		Encoders getEncoders();
		void resetEncoders();
		EncoderTicks getEncoderTicks();

		EncoderScales getWheelDiamScales();
		EncoderScales getWheelTrackScales();

		bool isUsingInertial();

	private:
		static std::shared_ptr<Odometry> odomSingleton;
		Odometry(Encoders encs, EncoderScales wheelDiamScales, EncoderScales wheelTrackScales);
		Odometry(Encoders encs, EncoderScales wheelDiamScales, EncoderScales wheelTrackScales, std::shared_ptr<pros::Imu> inertial);

		Pose pose { 0, 0, 0, 0 };

		Encoders encs;
		EncoderScales wheelDiamScales;
		EncoderScales wheelTrackScales;

		const int MAX_TICKS = 1000;

		bool useInertial { false };
		std::shared_ptr<pros::Imu> inertial { nullptr };

		pros::task_t odomTask;

		static void odomManager(void* param);

		void step(EncoderVals encDelta);
};
} // namespace lib16868X