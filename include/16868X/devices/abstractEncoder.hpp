#pragma once

namespace lib16868X {
class AbstractEncoder {
	public:
		virtual double get() = 0;
		virtual void resetZero() = 0;
		virtual double getVelocity() = 0;

		inline int getTPR() {
			return tpr;
		}
	
	protected:
		int tpr;
};
} // namespace lib16868X