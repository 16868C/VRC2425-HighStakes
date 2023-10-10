#include "16868Z/util/filters/rangeExtremaFilter.hpp"
#include "16868Z/util/util.hpp"
#include <algorithm>
#include <vector>

using namespace lib16868C;

RangeExtremaFilter::RangeExtremaFilter(int sampleSize) : sampleSize(sampleSize) {}

double RangeExtremaFilter::filter(double input) {
	samples.push(input);
	if (samples.size() > sampleSize) samples.pop();

	std::vector<double> v = Util::queueToVector(samples);
	return output = *std::max_element(v.begin(), v.end());
}