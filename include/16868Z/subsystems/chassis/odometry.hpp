#pragma once
#include "16868Z/devices/abstractEncoder.hpp"
#include "16868Z/util/pose.hpp"
#include "16868Z/util/util.hpp"
#include "api.h"
#include <memory>

namespace lib16868C {
enum class OdomType {
	THREE_ENCODER,
	TWO_ENCODER,
	ACCEL
};

enum class EncoderValsType {
	DISTANCE,
	VELOCITY,
	ACCELERATION,
	TICKS,
	SCALES_WHEEL_DIAM,
	SCALES_WHEEL_TRACK
};

typedef struct EncoderVals EncoderScales, EncoderTicks, Deltas;
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

	inline EncoderVals toDistance(EncoderVals wheelDiamScales, int tpr) {
		if (type != EncoderValsType::TICKS) return *this;
		if (wheelDiamScales.type != EncoderValsType::SCALES_WHEEL_DIAM) return *this;

		return {left / static_cast<double>(tpr) * wheelDiamScales.left * M_PI,
				right / static_cast<double>(tpr) * wheelDiamScales.right * M_PI,
				rear / static_cast<double>(tpr) * wheelDiamScales.rear * M_PI,
				EncoderValsType::DISTANCE};
	}
};
inline EncoderVals operator/(const double& divident, const EncoderVals& divisor) {
	return {divident / divisor.left, divident / divisor.right, divident / divisor.rear, divisor.theta, divisor.type};
}

struct OdomSensors {
	std::shared_ptr<AbstractEncoder> left { nullptr };
	std::shared_ptr<AbstractEncoder> right { nullptr };
	std::shared_ptr<AbstractEncoder> rear { nullptr };
	std::shared_ptr<pros::Imu> inertial { nullptr };

	inline Deltas getDeltas(OdomType type) {
		switch(type) {
			case OdomType::THREE_ENCODER:
				return {left->get(), right->get(), rear->get(), EncoderValsType::TICKS};
			case OdomType::TWO_ENCODER:
				return {left->get(), 0, rear->get(), EncoderValsType::TICKS};
			case OdomType::ACCEL: {
				pros::c::imu_accel_s_t a = inertial->get_accel();
				return {a.x, 0, a.z, EncoderValsType::ACCELERATION}; }
			default:
				return {0, 0, 0, EncoderValsType::TICKS};
		}
	}
	inline void reset() {
		if (left) left->resetZero();
		if (right) right->resetZero();
		if (rear) rear->resetZero();
		if (inertial) inertial->reset();
	}
};

class Odometry {
	public:
		Odometry(OdomType type, OdomSensors snrs, EncoderScales wheelDiamScales, EncoderScales wheelTrackScales);

		void init();
		void init(Pose pose);

		Pose getPose();

		OdomType getOdomType();
		OdomSensors getSensors();
		void resetEncoders();
		Deltas getDeltas();

		EncoderScales getWheelDiamScales();
		EncoderScales getWheelTrackScales();

		EncoderScales calibWheelDiam(double actualDist);
		EncoderScales calibWheelTrack(double actualAng);

	private:
		Pose pose { 0, 0, 0, 0 };

		OdomType odomType;
		OdomSensors snrs;

		EncoderScales wheelDiamScales;
		EncoderScales wheelTrackScales;

		pros::task_t odomTask;
		static void odomManager(void* param);

		const int MAX_DELTA = 1000;

		void step(EncoderVals encDelta);
};
} // namespace lib16868C