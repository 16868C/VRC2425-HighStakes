#pragma once
#include <functional>

namespace lib16868C {
struct PIDGains {
	double kP, kI, kD, kF = 0;
};

class PIDController {
	public:
		PIDController(PIDGains gains, double outputMax = 1, double outputMin = -1, double maxIntegral = 1e5, double integralRange = 15, bool resetIntegralOnCross = true, std::function<bool()> settleCond = {[]() { return false; }});
		PIDController(const PIDController& pidController);

		void setGains(PIDGains gains);
		void setOutputMaxMin(double max, double min);
		void setMaxIntegral(double max);
		void setResetIntegralOnCross(bool resetIntegralOnCross);
		void setSettleCondition(std::function<bool()> settleCond);

		double calculate(double error);
		double calculate(double target, double curent);

		bool isSettled();

	private:
		PIDGains gains;
		double outputMax, outputMin;
		double maxIntegral;
		double integralRange;
		bool resetIntegralOnCross;
		double output;
		double prevError = 0;
		double integral = 0;
		double prevTime = 0;

		std::function<bool()> settleCond;
};
} // namespace lib16868C