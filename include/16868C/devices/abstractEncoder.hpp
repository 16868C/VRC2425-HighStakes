#pragma once

namespace lib16868C {
class AbstractEncoder {
	public:
		virtual double get() = 0;
		virtual void resetZero() = 0;
		virtual double getVelocity() = 0;

		int getTPR();
	
	protected:
		int tpr;
};
} // namespace lib16868C