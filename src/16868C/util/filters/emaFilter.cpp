#include "16868C/util/filters/emafilter.hpp"

using namespace lib16868C;

EMAFilter::EMAFilter(double alpha) : alpha(alpha) {}

double EMAFilter::filter(double input, double alpha) {
	return prev = alpha * input + (1 - alpha) * prev;
}