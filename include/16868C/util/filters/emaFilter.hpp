#pragma once

namespace lib16868C {
class EMAFilter {
	public:
		EMAFilter(double alpha = 0.5);
		double filter(double input);
		double filter(double input, double alpha);
		double getOutput();
		void setAlpha(double alpha);

	private:
		double alpha;
		double prev = 0;
};
} // namespace lib16868C