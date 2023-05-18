#include "16868X/util/filters/rangeExtremaFilter.hpp"
#include "16868X/util/util.hpp"
#include <vector>
#include <algorithm>

using namespace lib16868X;

RangeExtremaFilter::RangeExtremaFilter(int sampleSize) : sampleSize(sampleSize) {}

double RangeExtremaFilter::filter(double input) {
	samples.push(input);
	if (samples.size() > sampleSize) samples.pop();

	std::vector<double> v = queueToVector(samples);
	return output = *std::max_element(v.begin(), v.end());
}