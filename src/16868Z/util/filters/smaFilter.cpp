#include "16868Z/util/filters/smafilter.hpp"

using namespace lib16868C;

SMAFilter::SMAFilter(int sampleSize) : sampleSize(sampleSize) {}

double SMAFilter::filter(double input) {
	sum += input;
	samples.push(input);
	if (samples.size() > sampleSize) {
		sum -= samples.front();
		samples.pop();
	}
	return output = sum / static_cast<double>(samples.size());
}