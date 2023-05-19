#include "16868X/devices/motor.hpp"
#include "16868X/util/util.hpp"

using namespace lib16868X;

Motor::Motor(const Motor& mtr) : okapi::Motor(mtr) {
	copied = true;
	Motor::addMotor(std::make_shared<Motor>(*this));

	cartridge = gearsetToCartridge(okapi::Motor::getGearing());

	tpr = mtr.tpr;

	if (mtrManagerTask == NULL)
		mtrManagerTask = pros::c::task_create(mtrManager, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Motor Manager");
}
Motor::Motor(int port, Cartridge cartridge)
			: okapi::Motor(port,
							(Util::sgn(port) == 1 ? true : false),
							Motor::cartridgeToGearset(cartridge),
							okapi::AbstractMotor::encoderUnits::counts) {
	Motor::addMotor(std::make_shared<Motor>(*this));

	this->cartridge = cartridge;

	// Maps the cartridge to the tpr proportionally, proof of the graph: https://www.desmos.com/calculator/yq3xm5fxzm
	tpr = 18000 / static_cast<int>(cartridge);

	if (mtrManagerTask == NULL)
		mtrManagerTask = pros::c::task_create(mtrManager, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Motor Manager");
}