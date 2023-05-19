#pragma once
#include <functional>

namespace lib16868X {
struct PIDGains {
	double kP, kI, kD, kF = 0;
};

class PIDController {
	public:
		PIDController(const PIDController& pidController);
		PIDController(PIDGains gains, double target, double outputMax = 1, double outputMin = -1, double maxIntegral = 1e5, double integralRange = 15, bool resetIntegralOnCross = true, std::function<bool()> settleCond = {[]() { return false; }});

		inline void setGains(PIDGains gains) {
			this->gains = gains;
		}
		inline void setOutputMaxMin(double outputMax, double outputMin) {
			this->outputMax = outputMax;
			this->outputMin = outputMin;
		}
		inline void setMaxIntegral(double maxIntegral) {
			this->maxIntegral = maxIntegral;
		}
		inline void setResetIntegralOnCross(bool resetIntegralOnCross) {
			this->resetIntegralOnCross = resetIntegralOnCross;
		}
		inline void setTarget(double target) {
			this->target = target;
		}
		inline void setSettleCondition(std::function<bool()> settleCond) {
			this->settleCond = settleCond;
		}

		double calculate(double input);

		inline bool isSettled() {
			return settleCond();
		}

	private:
		PIDGains gains;
		double outputMax, outputMin;
		double maxIntegral;
		double integralRange;
		bool resetIntegralOnCross;
		double target;
		double output;
		double prevError = 0;
		double integral = 0;
		double prevTime = 0;

		std::function<bool()> settleCond {[]() { return false; }};
};
} // namespace lib16868X