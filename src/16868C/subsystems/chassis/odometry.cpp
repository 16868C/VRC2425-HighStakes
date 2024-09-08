#include "16868C/subsystems/chassis/odometry.hpp"
#include "16868C/devices/inertial.hpp"
#include "16868C/util/logger.hpp"
#include "16868C/util/math.hpp"
#include "16868C/util/util.hpp"
#include "pros/rtos.hpp"
#include <iostream>

using namespace lib16868C;
using namespace okapi::literals;

/* -------------------------------------------------------------------------- */
/*                               Distance Sensor                              */
/* -------------------------------------------------------------------------- */
DistanceSensor::DistanceSensor() {}
DistanceSensor::DistanceSensor(okapi::DistanceSensor* snsr, okapi::QLength offset) : snsr(snsr) {
	this->offset = offset.convert(okapi::millimeter);
}

double DistanceSensor::getDist() const {
	if (snsr) return snsr->get() + offset;
	return 0;
}
double DistanceSensor::getConfidence() const {
	if (snsr) return snsr->getConfidence();
	return 0;
}

/* -------------------------------------------------------------------------- */
/*                                  Odometry                                  */
/* -------------------------------------------------------------------------- */

/* -------------------------------- Main Loop ------------------------------- */
void Odometry::odomManager(void* param) {
	std::array<double, 4> prev;
	prev.fill(0);

	Odometry* odom = static_cast<Odometry*>(param);
	
	uint32_t time = pros::millis();
	int n = 1;
	while (true) {
		// Distance and theta
		std::array<double, 4> curr;
		for (int i = 0; i < 3; i++) curr[i] = odom->trackingWheels[i]->getDist();
		if (!odom->inertial) curr[3] = (curr[1] - curr[0]) / (odom->trackingWheels[0]->getOffset() + odom->trackingWheels[1]->getOffset());
		else curr[3] = odom->inertial->get_rotation(AngleUnit::RAD);

		// Deltas
		std::array<double, 4> deltas;
		for (int i = 0; i < 4; i++) deltas[i] = curr[i] - prev[i];

		odom->step(deltas);

		// Output
		Pose pose = odom->getPose();
		pros::lcd::print(0, "X: %.2f, Y: %.2f", pose.x, pose.y);
		pros::lcd::print(1, "Deg: %.2f, Rad: %.2f", Util::radToDeg(pose.theta), pose.theta);

		std::array<TrackingWheel*, 3> encs = odom->trackingWheels;
		std::array<DistanceSensor*, 4> dists = odom->distanceSensors;
		// if (n++ % 5 == 0) {
		// 	n = 1;
		// printDebug("%s, %f, %f\n", pose.toStr(), encs[0]->getDist(), encs[2]->getDist());
		// }
		if (encs[1]->getType() != TrackingWheelType::INVALID) pros::lcd::print(2, "Left: %.2f, Right: %.2f", encs[0]->getDist(), encs[1]->getDist());
		else pros::lcd::print(2, "Forward: %.2f", encs[0]->getDist());
		if (odom->inertial) pros::lcd::print(3, "Middle: %.2f, Theta: %.2f", encs[2]->getDist(), odom->inertial->get_rotation(AngleUnit::DEG));
		else pros::lcd::print(3, "Middle: %.2f", encs[2]->getDist());

		prev = curr;

		pros::Task::delay_until(&time, 10);
	}
}

/* ------------------------------ Constructors ------------------------------ */
Odometry::Odometry() {}
Odometry::Odometry(std::array<TrackingWheel, 3> trackingWheels, std::array<DistanceSensor, 4> distanceSensors, Inertial* inertial)
				: inertial(inertial) {
	leftEnc = trackingWheels[0];
	rightEnc = trackingWheels[1];
	middleEnc = trackingWheels[2];
	this->trackingWheels = { &leftEnc, &rightEnc, &middleEnc };

	frontDist = distanceSensors[0];
	rightDist = distanceSensors[1];
	rearDist = distanceSensors[2];
	leftDist = distanceSensors[3];
	this->distanceSensors = { &frontDist, &leftDist, &rearDist, &rightDist };

	odomTask.suspend();
}
Odometry::Odometry(TrackingWheel left, TrackingWheel right, TrackingWheel middle)
	: leftEnc(left), rightEnc(right), middleEnc(middle) {
	this->trackingWheels = { &leftEnc, &rightEnc, &middleEnc };

	odomTask.suspend();
}
Odometry::Odometry(TrackingWheel left, TrackingWheel right, TrackingWheel middle, Inertial* inertial)
	: leftEnc(left), rightEnc(right), middleEnc(middle), inertial(inertial) {
	this->distanceSensors = { &frontDist, &leftDist, &rearDist, &rightDist };

	odomTask.suspend();
}
Odometry::Odometry(DistanceSensor front, DistanceSensor right, DistanceSensor rear, DistanceSensor left, Inertial* inertial)
	: frontDist(front), rightDist(right), rearDist(rear), leftDist(left), inertial(inertial) {
	this->distanceSensors = { &frontDist, &leftDist, &rearDist, &rightDist };

	odomTask.suspend();
}
Odometry::Odometry::Odometry(Odometry& odom) {
	leftEnc = odom.leftEnc;
	rightEnc = odom.rightEnc;
	middleEnc = odom.middleEnc;
	this->trackingWheels = { &leftEnc, &rightEnc, &middleEnc };

	frontDist = odom.frontDist;
	rightDist = odom.rightDist;
	rearDist = odom.rearDist;
	leftDist = odom.leftDist;
	this->distanceSensors = { &frontDist, &leftDist, &rearDist, &rightDist };

	inertial = odom.inertial;

	odomTask.suspend();
}

/* --------------------------- Initialize Methods --------------------------- */
void Odometry::init() {
	init({ 0_in, 0_in, 0_rad, 0 });
}
void Odometry::init(Pose pose) {
	// Resetting sensors
	pros::delay(500); // Just in case the sensors have not been initialized yet
	for (int i = 0; i < 3; i++) trackingWheels[i]->reset();
	inertial->calibrate();

	// Resetting pose
	update(pose);
	inertial->set_rotation(pose.theta * okapi::radian);

	// Starting task
	odomTask.resume();
}

/* -------------------------- Pose Related Methods -------------------------- */
Pose Odometry::getPose() {
	if (!poseMutex.take(50)) {
		std::cerr << "[Odometry::getPose] Mutex timeout - unable to read current pose" << std::endl;
		return prevPose;
	}
	// while (!poseMutex.take(5)) {
	// 	pros::delay(1);
	// }

	Pose pos = pose;
	poseMutex.give();
	prevPose = pos;
	return pos;
}
Pose Odometry::getState() {
	return getPose();
}

void Odometry::update(bool front, bool right, bool rear, bool left) {
	std::array<bool, 4> snsrUse = {front, right, rear, left};

	double theta = std::abs(ReduceAngle::radPi2(getPose().theta));
	if (theta > M_PI / 12.0 && theta < M_PI * 5 / 12.0) { // Not perpendicular to wall
		return;
	}
	theta = ReduceAngle::rad2Pi(getPose().theta);
	if (theta == M_PI_2 || theta == M_PI_2 * 3) theta -= 1e-5; // Avoid tan(90) and tan(270) (Divide by zero error)
	
	double dir = round(theta / M_PI_2);
	if (dir == 4) dir = 0;

	// Determine which distance sensor corresponds to which direction
	std::array<DistanceSensor*, 4> dirDists { nullptr, nullptr, nullptr, nullptr };
	int j = dir;
	for (int i = 0; i < 4; i++, j++) {
		if (j == 4) j = 0;

		if (snsrUse[j]) dirDists[i] = distanceSensors[j];
	}

	// Calculate the robot's position from each side of the robot
	theta = ReduceAngle::reduce(getPose().theta, M_PI / 4, -M_PI / 4);
	std::pair<double, double> x1, x2, y1, y2;
	if (dirDists[0]) x1 = {(12_ft).convert(okapi::millimeter) - dirDists[0]->getDist() * std::abs(std::cos(theta)), dirDists[0]->getConfidence()};
	if (dirDists[2]) x2 = {dirDists[2]->getDist() * std::abs(std::cos(theta)), dirDists[2]->getConfidence()};
	if (dirDists[1]) y1 = {dirDists[1]->getDist() * std::abs(std::cos(theta)), dirDists[1]->getConfidence()};
	if (dirDists[3]) y2 = {(12_ft).convert(okapi::millimeter) - dirDists[3]->getDist() * std::abs(std::cos(theta)), dirDists[3]->getConfidence()};
	std::cout << dirDists[0]->getDist() << " " << dirDists[2]->getDist() << " " << dirDists[1]->getDist() << " " << dirDists[3]->getDist() << "\n";

	// Use the most accurate readings
	Pose newPose(x1.first * okapi::millimeter, y1.first * okapi::millimeter, inertial->get_rotation(AngleUnit::RAD) * okapi::radian, pros::millis());
	if (x2.second > x1.second) newPose.x = (x2.first * okapi::millimeter).convert(okapi::inch);
	if (y2.second > y1.second) newPose.y = (y2.first * okapi::millimeter).convert(okapi::inch);
	update(newPose);
}
void Odometry::update(okapi::QLength x, okapi::QLength y) {
	// if (!poseMutex.take(50)) {
	// 	std::cerr << "[Odometry::update] Mutex timout - unable to update pose" << std::endl;
	// 	return;
	// }
	while (!poseMutex.take(5)) {
		pros::delay(1);
	}
	pose = Pose(x, y, pose.theta * okapi::radian, pose.time);
	poseMutex.give();
}
void Odometry::update(okapi::QLength x, okapi::QLength y, okapi::QAngle theta) {
	// inertial->set_rotation(theta);
	if (!poseMutex.take(50)) {
		std::cerr << "[Odometry::update] Mutex timout - unable to update pose" << std::endl;
		return;
	}
	// while (!poseMutex.take(5)) {
	// 	pros::delay(1);
	// }
	// this->pose = Pose(x, y, theta, pros::millis());
	pose.x = x.convert(okapi::inch);
	pose.y = y.convert(okapi::inch);
	pose.theta = theta.convert(okapi::radian);
	pose.time = pros::millis();
	// std::cout << x.convert(okapi::inch) << " " << y.convert(okapi::inch) << "\n";
	poseMutex.give();
}
void Odometry::update(Pose pose) {
	update(pose.x * okapi::inch, pose.y * okapi::inch, pose.theta * okapi::radian);
}

/* --------------------------- Getters and Setter --------------------------- */
std::array<TrackingWheel*, 3> Odometry::getEncoders() const {
	return trackingWheels;
}
std::array<DistanceSensor*, 4> Odometry::getDistanceSensors() const {
	return distanceSensors;
}
Inertial* Odometry::getInertial() const {
	return inertial;
}

void Odometry::resetSensors() {
	for (int i = 0; i < 3; i++) trackingWheels[i]->reset();
	inertial->reset(true);
}

/* ---------------------------- Main calculations --------------------------- */
void Odometry::step(std::array<double, 4> deltas) {
	for (double d : deltas) {
		if (std::abs(d) > MAX_DELTA) {
			std::cerr << "Odometry delta too large: " << d << "\n";
			return;
		}
	}

	// Delta Distances
	double deltaL = deltas[0];
	double deltaR = deltas[1];
	double deltaM = deltas[2];
	double deltaA = deltas[3];

	// Local offsets
	double localOffsetX = deltaM;
	double localOffsetY = deltaL;
	if (deltaA != 0) {
		// std::cout << deltaM / deltaA + trackingWheels[2]->getOffset() << " " << deltaL / deltaA + trackingWheels[0]->getOffset() << "\n";
		localOffsetX = 2 * std::sin(deltaA / 2.0) * (deltaM / deltaA + trackingWheels[2]->getOffset());
		localOffsetY = 2 * std::sin(deltaA / 2.0) * (deltaL / deltaA + trackingWheels[0]->getOffset());
	}

	// Adding the x and y components of each of the local offsets to calculate the global offsets
	double avgA = pose.theta + (deltaA / 2.0);
	// double globalDeltaX = localOffsetX * cos(avgA) + localOffsetY * cos(avgA);
	// double globalDeltaY = localOffsetX * -sin(avgA) + localOffsetY * sin(avgA);
	double globalDeltaX = localOffsetX * -sin(avgA) + localOffsetY * cos(avgA);
	double globalDeltaY = localOffsetX * cos(avgA) + localOffsetY * sin(avgA);

	if (std::isnan(globalDeltaX)) globalDeltaX = 0;
	if (std::isnan(globalDeltaY)) globalDeltaY = 0;
	if (std::isnan(deltaA)) deltaA = 0;

	double globalX = pose.x + globalDeltaX;
	double globalY = pose.y + globalDeltaY;
	double globalTheta = inertial->get_rotation(AngleUnit::RAD);
	update({globalX * okapi::inch, globalY * okapi::inch, globalTheta * okapi::radian, pros::millis()});
}