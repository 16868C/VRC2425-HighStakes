#include "16868C/subsystems/chassis/odometry.hpp"
#include "16868C/util/logger.hpp"
#include "16868C/util/math.hpp"
#include <limits>

using namespace lib16868C;
using namespace okapi::literals;

/* -------------------------------------------------------------------------- */
/*                               Distance Sensor                              */
/* -------------------------------------------------------------------------- */
DistanceSensor::DistanceSensor() {}
DistanceSensor::DistanceSensor(okapi::DistanceSensor* snsr, okapi::QLength offset) : snsr(snsr) {
	this->offset = offset.convert(okapi::inch);
}

double DistanceSensor::getDist() const {
	if (snsr) return snsr->get();
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
		pros::lcd::print(0, "X: %.2f, Y: %.2f", *pose.x(), *pose.y());
		pros::lcd::print(1, "Deg: %.2f, Rad: %.2f", Util::radToDeg(*pose.theta()), *pose.theta());

		std::array<TrackingWheel*, 3> encs = odom->trackingWheels;
		std::array<DistanceSensor*, 4> dists = odom->distanceSensors;
		if (encs[1]->getType() != TrackingWheelType::INVALID) pros::lcd::print(2, "Left: %.2f, Right: %.2f", encs[0]->getDist(), encs[1]->getDist());
		else pros::lcd::print(2, "Forward: %.2f", encs[1]->getDist());
		if (odom->inertial) pros::lcd::print(3, "Middle: %.2f, Theta: %.2f", encs[2]->getDist(), odom->inertial->get_rotation(AngleUnit::DEG));
		else pros::lcd::print(3, "Middle: %.2f", encs[2]->getDist());

		prev = curr;

		pros::Task::delay_until(&time, 10);
	}
}

/* ------------------------------ Constructors ------------------------------ */
Odometry::Odometry() {}
Odometry::Odometry(std::array<TrackingWheel, 3> trackingWheels, std::array<DistanceSensor, 4> distanceSensors, Inertial* inertial) : inertial(inertial) {
	leftEnc = trackingWheels[0];
	rightEnc = trackingWheels[1];
	middleEnc = trackingWheels[2];

	frontDist = distanceSensors[0];
	rightDist = distanceSensors[1];
	rearDist = distanceSensors[2];
	leftDist = distanceSensors[3];
}
Odometry::Odometry(TrackingWheel left, TrackingWheel right, TrackingWheel middle)
	: leftEnc(left), rightEnc(right), middleEnc(middle) {
}
Odometry::Odometry(TrackingWheel left, TrackingWheel right, TrackingWheel middle, Inertial* inertial)
	: leftEnc(left), rightEnc(right), middleEnc(middle), inertial(inertial) {
}
Odometry::Odometry(DistanceSensor front, DistanceSensor right, DistanceSensor rear, DistanceSensor left, Inertial* inertial)
	: frontDist(front), rightDist(right), rearDist(rear), leftDist(left), inertial(inertial) {
}
Odometry::Odometry::Odometry(Odometry& odom) {
	leftEnc = odom.leftEnc;
	rightEnc = odom.rightEnc;
	middleEnc = odom.middleEnc;

	frontDist = odom.frontDist;
	rightDist = odom.rightDist;
	rearDist = odom.rearDist;
	leftDist = odom.leftDist;

	inertial = odom.inertial;
}

/* --------------------------- Initialize Methods --------------------------- */
void Odometry::init() {
	init({ 0_in, 0_in, 0_rad, 0 });
}
void Odometry::init(Pose pose) {
	// Resetting sensors
	pros::delay(100); // Just in case the sensors have not been initialized yet
	for (int i = 0; i < 3; i++) trackingWheels[i]->reset();
	inertial->calibrate();

	// Resetting pose
	this->pose = pose;

	// Starting task
	odomTask = pros::c::task_create(odomManager, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odometry");
}

/* -------------------------- Pose Related Methods -------------------------- */
Pose Odometry::getPose() {
	if (!poseMutex.take(50)) {
		std::cerr << "[Odometry::getPose] Mutex timeout - unable to read current pose" << std::endl;
		return prevPose;
	}

	Pose pos = pose;
	poseMutex.give();
	prevPose = pos;
	return pos;
}
Pose Odometry::getState() {
	return getPose();
}

void Odometry::update(bool front, bool right, bool back, bool left) {
	std::array<Pose, 5> newPose;

	Pose curPose = getPose();

	double theta = ReduceAngle::radPi2(inertial->get_rotation(AngleUnit::RAD));
	if (std::abs(theta) > M_PI / 12.0 && std::abs(theta) < M_PI * 5 / 12.0) return; // Not perpendicular to wall
	theta = ReduceAngle::rad2Pi(inertial->get_rotation(AngleUnit::RAD));
	if (theta == M_PI_2 || theta == M_PI_2 * 3) theta -= 1e-5; // Avoid tan(90) and tan(270)

	std::array<double, 4> distReads;
	for (int i = 0; i < 4; i++) distReads[i] = distanceSensors[i]->getDist();
	std::array<int, 4> distConf;
	for (int i = 0; i < 4; i++) distConf[i] = distanceSensors[i]->getConfidence();
	std::array<bool, 4> useSnsr {front, right, back, left};

	for (int i = 0; i < 4; i++) {
		if (std::isnan(distReads[i]) || !useSnsr[i]) continue; // Not using the sensor
		if (distReads[i] > FIELD_WIDTH || distReads[i] < 0) continue; // Read error
		if (distConf[i] < MIN_CONFIDENCE) continue; // Confidence too low

		double distTheta = theta + i * M_PI_2;
		Line dist(std::tan(distTheta), *curPose.pos());
		double distToWall = distReads[i] + distanceSensors[i]->offset, newPos = distToWall;
		for (double a = 0; a <= 2 * M_PI; a += M_PI_2) {
			if (a == 0 && std::abs(distTheta) < ReduceAngle::rad2Pi(a - M_PI / 12.0) && std::abs(distTheta) > ReduceAngle::rad2Pi(a + M_PI / 12.0)) continue; // Not within 15 deg of an axis
			else if (a != 0 && (std::abs(distTheta) < ReduceAngle::rad2Pi(a - M_PI / 12.0) || std::abs(distTheta) > ReduceAngle::rad2Pi(a + M_PI / 12.0))) continue; // Not within 15 deg of an axis

			if (walls[a].isInsideSegment(walls[a].getIntersection(dist))) { // Distance sensor is reading distance to correct wall
				distToWall *= std::abs(std::sin(distTheta + a + M_PI_2));
				newPos = distToWall;
				
				*getDistUpdateCoord(distTheta, i, newPose, distConf) = newPos;
			}
		}
	}

	newPose[0].setX(*newPose[3].x() > *newPose[4].x() ? *newPose[1].x() : *newPose[2].x());
	newPose[0].setY(*newPose[3].y() > *newPose[4].y() ? *newPose[1].y() : *newPose[2].y());
	newPose[0].setTheta(inertial->get_rotation(AngleUnit::RAD));
	if (std::isnan(*newPose[0].x()) || std::isnan(*newPose[0].y())) return;
	update(newPose[0]);
}
void Odometry::update(Pose pose) {
	if (!poseMutex.take(50)) {
		std::cerr << "[Odometry::update] Mutex timout - unable to update pose" << std::endl;
		return;
	}
	this->pose = pose;
	poseMutex.give();
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
	inertial->reset();
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
		localOffsetX = 2 * std::sin(deltaA / 2.0) * (deltaM / deltaA - trackingWheels[2]->getOffset());
		localOffsetY = 2 * std::sin(deltaA / 2.0) * (deltaL / deltaA - trackingWheels[0]->getOffset());
	}

	// Adding the x and y components of each of the local offsets to calculate the global offsets
	double avgA = *pose.theta() + (deltaA / 2.0);
	double globalDeltaX = localOffsetX * -sin(avgA) + localOffsetY * sin(avgA);
	double globalDeltaY = localOffsetX * cos(avgA) + localOffsetY * cos(avgA);

	if (std::isnan(globalDeltaX)) globalDeltaX = 0;
	if (std::isnan(globalDeltaY)) globalDeltaY = 0;
	if (std::isnan(deltaA)) deltaA = 0;

	double globalX = *pose.x() + globalDeltaX;
	double globalY = *pose.y() + globalDeltaY;
	double globalTheta = inertial->get_rotation(AngleUnit::RAD);
	update({globalX * okapi::inch, globalY * okapi::inch, globalTheta * okapi::radian, pros::millis()});
}

/* ----------------------------- Utility Methods ---------------------------- */
double* Odometry::getDistUpdateCoord(double a, int i, std::array<Pose, 5>& newPose, std::array<int, 4> confs) {
	if (std::abs(a - 0) <= 15 || std::abs(a - M_PI) <= 15) {
		switch(i) {
			case 0:
			case 2:
				newPose[0.5 * i + 3].setX(confs[i]);
				return (double*) newPose[0.5 * i + 1].x();
			case 1:
			case 3:
				newPose[0.5 * i + 2.5].setY(confs[i]);
				return (double*) newPose[0.5 * i + 0.5].y();
		}
	} else {
		switch(i) {
			case 0:
			case 2:
				newPose[0.5 * i + 3].setY(confs[i]);
				return (double*) newPose[0.5 * i + 1].y();
			case 1:
			case 3:
				newPose[0.5 * i + 2.5].setX(confs[i]);
				return (double*) newPose[0.5 * i + 0.5].x();
		}
	}
	return (double*) newPose[0].x();
}