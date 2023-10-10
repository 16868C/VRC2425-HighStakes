#pragma once

namespace lib16868C {
class EMAFilter {
	public:
		EMAFilter(double alpha = 0.5);
		inline double filter(double input) {
			return filter(input, alpha);
		}
		double filter(double input, double alpha);
		inline double getOutput() { return prev; }
		inline void setAlpha(double alpha) { this->alpha = alpha; };

	private:
		double alpha;
		double prev = 0;
};
} // namespace lib16868C