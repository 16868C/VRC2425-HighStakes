#include "16868Z/controllers/pidController.hpp"
#include "16868Z/util/util.hpp"
#include "api.h"
#include <algorithm>

using namespace lib16868Z;

PIDController::PIDController(const PIDController& pidController) {
	this->gains = pidController.gains;
	this->outputMax = pidController.outputMax;
	this->outputMin = pidController.outputMin;
	this->maxIntegral = pidController.maxIntegral;
	this->integralRange = pidController.integralRange;
	this->resetIntegralOnCross = pidController.resetIntegralOnCross;
	this->output = pidController.output;
	this->prevError = pidController.prevError;
	this->prevTime = pidController.prevTime;
}
PIDController::PIDController(PIDGains gains, double outputMax, double outputMin, double maxIntegral, double integralRange, bool resetIntegralOnCross, std::function<bool()> settleCond) {
	this->gains = gains;
	this->outputMax = outputMax;
	this->outputMin = outputMin;
	this->maxIntegral = std::abs(maxIntegral);
	this->integralRange = std::fabs(integralRange);
	this->resetIntegralOnCross = resetIntegralOnCross;
	this->output = 0;
	this->settleCond = settleCond;
}

double PIDController::calculate(double error) {
	uint32_t currTime = pros::millis();
	double dT = currTime - prevTime;
	prevTime = currTime;

	if (error <= integralRange) integral += error;
	double dError = error - prevError;
	double derivative = dError / dT;
	prevError = error;

	if (resetIntegralOnCross && Util::sgn(error) != Util::sgn(prevError)) integral = 0;
	integral = std::clamp(integral, -maxIntegral, maxIntegral);

	output = error * gains.kP + integral * gains.kI + derivative * gains.kD + gains.kF;
	if (outputMin != outputMax) output = std::clamp(output, outputMin, outputMax);
	return output;
}
double PIDController::calculate(double target, double current) {
	return calculate(target - current);
}