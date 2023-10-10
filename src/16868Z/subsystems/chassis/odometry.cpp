#include "16868Z/subsystems/chassis/odometry.hpp"

using namespace lib16868C;

void Odometry::odomManager(void* param) {
	EncoderVals prev = {EncoderValsType::TICKS};

	// EncoderVals vel = {EncoderValsType::VELOCITY}; // Only for accel odom

	Odometry* odom = static_cast<Odometry*>(param);

	uint32_t time = pros::millis(), prevTime;
	while (true) {
		Deltas delta = odom->snrs.getDeltas(odom->odomType);

		EncoderVals curr = { EncoderValsType::DISTANCE };
		switch (odom->odomType) {
			case OdomType::THREE_ENCODER: {
				// Chassis scales
				const double leftCircumference = odom->wheelDiamScales.left * M_PI;
				const double rightCircumference = odom->wheelDiamScales.right * M_PI;
				const double rearCircumference = odom->wheelDiamScales.rear * M_PI;
				const double wheelTrack = odom->wheelTrackScales.left + odom->wheelTrackScales.right;

				// Distance and theta
				curr = delta.toDistance(odom->wheelDiamScales, odom->snrs.left->getTPR());
				curr.theta = (curr.right - curr.left) / wheelTrack;
				curr.rear -= (curr.theta - prev.theta) * odom->wheelTrackScales.rear; }
			case OdomType::TWO_ENCODER: {
				// Chassis scales
				const double fowardCircumference = odom->wheelDiamScales.left * M_PI;
				const double rearCircumference = odom->wheelDiamScales.rear * M_PI;

				// Distance and theta
				curr = delta.toDistance(odom->wheelDiamScales, odom->snrs.left->getTPR());
				curr.theta = odom->snrs.inertial->get_rotation() - prev.theta;
				curr.rear -= (curr.theta - prev.theta) * odom->wheelTrackScales.rear; }
			case OdomType::ACCEL: { // Accel odom not complete
				// // Velocity
				// vel.left += delta.left;
				// vel.rear += delta.rear;
				
				// // Distance
				// curr.left += vel.left;
				// curr.rear += vel.rear;
				// curr.theta = odom->snrs.inertial->get_rotation() - prev.theta;
				// curr.rear -= curr.theta - prev.theta;
			}
		}

		odom->step(curr - prev);

		Pose pose = odom->getPose();
		pros::lcd::print(0, "X: %.2f, Y: %.2f", pose.x, pose.y);
		pros::lcd::print(1, "Deg: %.2f, Rad: %.2f", Util::radToDeg(pose.theta), pose.theta);

		pros::lcd::print(2, "Left: %.2f, Right: %.2f", curr.left, curr.right);
		pros::lcd::print(3, "Rear: %.2f", curr.rear);

		prev = curr;

		pros::Task::delay_until(&time, 10);
	}
}

Odometry::Odometry(OdomType odomType, OdomSensors snrs, EncoderScales wheelDiamScales, EncoderScales wheelTrackScales) 
				: odomType(odomType), snrs(snrs), wheelDiamScales(wheelDiamScales), wheelTrackScales(wheelTrackScales) {}

void Odometry::init() {
	// Resetting sensors
	snrs.reset();

	// Resetting pose
	pose = { 0, 0, 0, 0 };

	// Starting task
	odomTask = pros::c::task_create(odomManager, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odometry");
}
void Odometry::init(Pose pose) {
	// Resetting sensors
	snrs.reset();

	// Resetting pose
	this->pose = pose;

	// Starting task
	odomTask = pros::c::task_create(odomManager, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odometry");
}

Pose Odometry::getPose() {
	return pose;
}

OdomSensors Odometry::getSensors() {
	return snrs;
}
void Odometry::resetEncoders() {
	snrs.reset();
}
Deltas Odometry::getDeltas() {
	return snrs.getDeltas(odomType);
}

EncoderScales Odometry::getWheelDiamScales() {
	return wheelDiamScales;
}
EncoderScales Odometry::getWheelTrackScales() {
	return wheelTrackScales;
}

EncoderScales Odometry::calibWheelDiam(double actualDist) {
	if (odomType != OdomType::THREE_ENCODER || odomType != OdomType::TWO_ENCODER) return {0, 0, 0, EncoderValsType::SCALES_WHEEL_DIAM};

	EncoderTicks ticksTravelled = snrs.getDeltas(odomType);
	EncoderVals distTravelled = ticksTravelled.toDistance(wheelDiamScales, snrs.left->getTPR());
	EncoderVals ratios = distTravelled / actualDist;
	EncoderVals newScales = wheelDiamScales * ratios;

	std::cout << "Ticks travelled: " << ticksTravelled.left << ", " << ticksTravelled.right << ", " << ticksTravelled.rear << "\n";
	std::cout << "Distance travelled: " << distTravelled.left << ", " << distTravelled.right << ", " << distTravelled.rear << "\n";
	std::cout << "Ratios: " << ratios.left << ", " << ratios.right << ", " << ratios.rear << "\n";
	std::cout << "Calibrated wheel diameters: " << newScales.left << ", " << newScales.right << ", " << newScales.rear << "\n";
	return newScales;
}

EncoderScales Odometry::calibWheelTrack(double actualAng) {
	if (odomType != OdomType::THREE_ENCODER) return {0, 0, 0, EncoderValsType::SCALES_WHEEL_TRACK};

	EncoderTicks ticksTravelled = snrs.getDeltas(odomType);
	EncoderVals distTravelled = ticksTravelled.toDistance(wheelDiamScales, snrs.left->getTPR());
	EncoderVals angTurned =  distTravelled / wheelTrackScales;
	EncoderVals ratios = angTurned / actualAng;
	EncoderVals newScales = wheelTrackScales * ratios;

	std::cout << "Ticks travelled: " << ticksTravelled.left << ", " << ticksTravelled.right << ", " << ticksTravelled.rear << "\n";
	std::cout << "Distance travelled: " << distTravelled.left << ", " << distTravelled.right << ", " << distTravelled.rear << "\n";
	std::cout << "Angle turned: " << angTurned.left << ", " << angTurned.right << ", " << angTurned.rear << "\n";
	std::cout << "Ratios: " << ratios.left << ", " << ratios.right << ", " << ratios.rear << "\n";
	std::cout << "Calibrated wheel tracks: " << newScales.left << ", " << newScales.right << ", " << newScales.rear << "\n";
	return newScales;
}

void Odometry::step(Deltas delta) {
	if (delta.type != EncoderValsType::DISTANCE) return;
	if (std::abs(delta.left) > MAX_DELTA || std::abs(delta.right) > MAX_DELTA || std::abs(delta.rear) > MAX_DELTA) {
		std::cerr << "[Odometry] Sensor deltas too large: ";
		std::cerr << delta.left << ", ";
		if (delta.right) std::cerr << delta.right << " ";
		std::cerr << delta.rear << "\n";
		return;
	}

	// Local offsets
	double localOffsetX = delta.rear;
	double localOffsetY = (delta.left + delta.right) / 2.0;
	if (delta.left != delta.right) {
		localOffsetX = 2 * sin(delta.theta / 2.0) * (delta.rear / delta.theta + wheelTrackScales.rear);
		localOffsetY = 2 * sin(delta.theta / 2.0) * (delta.right / delta.theta + wheelTrackScales.right);
	}

	// Polar offsets to shift to global frame
	double avgA = pose.theta + (delta.theta / 2.0);

	double polarR = std::hypot(localOffsetX, localOffsetY);
	double polarA = std::atan2(localOffsetY, localOffsetX) - avgA;

	// Global offsets
	double globalDeltaX = polarR * cos(polarA);
	double globalDeltaY = polarR * sin(polarA);

	if (std::isnan(globalDeltaX)) globalDeltaX = 0;
	if (std::isnan(globalDeltaY)) globalDeltaY = 0;
	if (std::isnan(delta.theta)) delta.theta = 0;

	double globalX = pose.x + globalDeltaX;
	double globalY = pose.y + globalDeltaY;
	double globalTheta = pose.theta + delta.theta;
	pose = {globalX, globalY, globalTheta, pros::millis()};
}