#pragma once
#include "api.h"
#include "16868C/util/util.hpp"

namespace lib16868C {
class Pneumatic {
	public:
		Pneumatic(uint port, bool initState = false);
		Pneumatic(uint smartPort, uint port, bool initState = false);

		void extend();
		void retract();
		void toggle();

		bool getState();

	private:
		pros::ADIDigitalOut pneumatic;
		bool state;

		void setState(bool state);
};
} // namespace lib16868C