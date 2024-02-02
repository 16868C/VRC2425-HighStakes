#include "16868C/subsystems/chassis/odometry.hpp"
#include "16868C/util/math.hpp"
#include <limits>

using namespace lib16868C;
using namespace okapi::literals;

/** Distance Sensor **/
DistanceSensor::DistanceSensor() {}
DistanceSensor::DistanceSensor(okapi::DistanceSensor* snsr, okapi::QLength offset) : snsr(snsr) {
	this->offset = offset.convert(okapi::inch);
}

/** Odometry **/
void Odometry::odomManager(void* param) {
	std::array<double, 4> prev;
	prev.fill(0);

	Odometry* odom = static_cast<Odometry*>(param);

	uint32_t time = pros::millis();
	while (true) {
		// Distance and theta
		std::array<double, 4> curr = { odom->trackingWheels[0].getDist(), odom->trackingWheels[1].getDist(), odom->trackingWheels[2].getDist() };
		if (!odom->inertial) curr[3] = (curr[1] - curr[0]) / (odom->trackingWheels[0].getOffset() + odom->trackingWheels[1].getOffset());
		else curr[3] = odom->inertial->get_rotation(AngleUnit::RAD);

		std::vector<double> deltas;
		for (int i = 0; i < 4; i++) deltas.push_back(curr[i] - prev[i]);
		odom->step(deltas);

		Pose pose = odom->getPose();
		pros::lcd::print(0, "X: %.2f, Y: %.2f", pose.x, pose.y);
		pros::lcd::print(1, "Deg: %.2f, Rad: %.2f", Util::radToDeg(pose.theta), pose.theta);

		std::array<TrackingWheel, 3> encs = odom->trackingWheels;
		std::array<DistanceSensor, 4> dists = odom->distanceSensors;
		if (encs[1].getType() != TrackingWheelType::INVALID) pros::lcd::print(2, "Left: %.2f, Right: %.2f", encs[0].getDist(), encs[1].getDist());
		else pros::lcd::print(2, "Forward: %.2f", encs[1].getDist());
		if (odom->inertial) pros::lcd::print(3, "Middle: %.2f, Theta: %.2f", encs[2].getDist(), odom->inertial->get_rotation(AngleUnit::DEG));
		else pros::lcd::print(3, "Middle: %.2f", encs[2].getDist());

		prev = curr;

		pros::Task::delay_until(&time, 10);
	}
}

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
	: leftEnc(left), rightEnc(right), middleEnc(middle) {}
Odometry::Odometry(TrackingWheel left, TrackingWheel right, TrackingWheel middle, Inertial* inertial)
	: leftEnc(left), rightEnc(right), middleEnc(middle), inertial(inertial) {}
Odometry::Odometry(DistanceSensor front, DistanceSensor right, DistanceSensor rear, DistanceSensor left, Inertial* inertial)
	: frontDist(front), rightDist(right), rearDist(rear), leftDist(left), inertial(inertial) {}
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

void Odometry::init() {
	init({ 0_in, 0_in, 0_rad, 0 });
}
void Odometry::init(Pose pose) {
	// Resetting sensors
	pros::delay(100); // Just in case the sensors have not been initialized yet
	for (int i = 0; i < 3; i++) trackingWheels[i].reset();

	// Resetting pose
	this->pose = pose;

	// Starting task
	odomTask = pros::c::task_create(odomManager, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odometry");
}

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
	if (theta == M_PI_2 || theta == M_PI_2 * 3) theta -= 1e-5;

	std::array<double, 4> distReads;
	for (int i = 0; i < 4; i++) distReads[i] = distanceSensors[i].getDist();

	std::array<int, 4> distConf;
	for (int i = 0; i < 4; i++) distConf[i] = distanceSensors[i].getConfidence();
	
	std::array<bool, 4> useSnsr {front, right, back, left};

	for (int i = 0; i < 4; i++) std::cout << i << ": (" << distReads[i] << ", " << distConf[i] << ") ";
	std::cout << "\n";

	for (int i = 0; i < 4; i++) {
		if (std::isnan(distReads[i]) || !useSnsr[i]) continue; // Not using the sensor
		if (distReads[i] > FIELD_WIDTH || distReads[i] < 0) continue; // Read error
		if (distConf[i] < MIN_CONFIDENCE) continue; // Confidence too low

		double distTheta = theta + i * M_PI_2;
		Line dist(std::tan(distTheta), curPose.pos);
		// std::cout << i << " " << dist.toStr() << "\n";
		double distToWall = distReads[i] + distanceSensors[i].offset, newPos = distToWall;
		for (double a = 0; a <= 2 * M_PI; a += M_PI_2) {
			// std::cout << a << "\n";
			// std::cout << a << " " << distTheta << " " << ReduceAngle::rad2Pi(a - M_PI / 12.0) << " " << ReduceAngle::rad2Pi(a + M_PI / 12.0) << "\n";
			if (a == 0 && std::abs(distTheta) < ReduceAngle::rad2Pi(a - M_PI / 12.0) && std::abs(distTheta) > ReduceAngle::rad2Pi(a + M_PI / 12.0)) continue; // Not within 15 deg of an axis
			else if (a != 0 && (std::abs(distTheta) < ReduceAngle::rad2Pi(a - M_PI / 12.0) || std::abs(distTheta) > ReduceAngle::rad2Pi(a + M_PI / 12.0))) continue; // Not within 15 deg of an axis

			// std::cout << " " << walls[a].getLine().toStr() << "\n";
			// std::cout << walls[a].getIntersection(dist).toStr() << "\n";
			if (walls[a].isInsideSegment(walls[a].getIntersection(dist))) { // Distance sensor is reading distance to correct wall
				distToWall *= std::abs(std::sin(distTheta + a + M_PI_2));
				newPos = distToWall;
				// std::cout << i << " " << a << " " << std::abs(std::sin(distTheta + a + M_PI_2)) << "\n";
				if (oppWall[std::round(theta / M_PI_2) * M_PI_2](i)) { std::cout << std::round(theta / M_PI_2) * M_PI_2 << "\n"; newPos = fieldWidth - distToWall; }

				getDistUpdateCoord(distTheta, i, newPose, distConf) = newPos;
				std::cout << newPose[1].toStr() << ", " << newPose[2].toStr() << "\n";
			}
		}
	}

	newPose[0].x = newPose[3].x > newPose[4].x ? newPose[1].x : newPose[2].x;
	newPose[0].y = newPose[3].y > newPose[4].y ? newPose[1].y : newPose[2].y;
	newPose[0].theta = inertial->get_rotation(AngleUnit::RAD);
	if (std::isnan(newPose[0].x) || std::isnan(newPose[0].y)) return;
	update(newPose[0]);

	/**
	for (int i = 0; i < 4; i++) {
		Line dist(std::tan(theta), curPose.pos);
		double distToWall = distReads[i] + distScales.getDist(i), newPos = distToWall;
		if (theta >= M_PI * 23 / 12.0 || theta <= M_PI / 12.0) { // Facing north wall; 345-15 deg
			if (north.isInsideSegment(north.getIntersection(dist))) {
				if (i < 2) distToWall *= std::abs(std::cos(theta));
				else distToWall *= std::abs(std::sin(theta));

				if (i == 0 || i == 3) newPos = fieldWidth - distToWall;
				
				switch (i) {
					case 0:
						if (newPos - curPose.y > MAX_DELTA.convert(okapi::inch) || newPos - curPose.y < 0) continue; // Delta too large or too small
						newPose.y = newPos;
						break;
					case 1:
						if (newPos - curPose.y > MAX_DELTA.convert(okapi::inch) || newPos - curPose.y < 0) continue; // Delta too large or too small
						if (distConf[1] > distConf[0]) newPose.y = newPos;
						break;
					case 2:
						if (newPos - curPose.x > MAX_DELTA.convert(okapi::inch) || newPos - curPose.x < 0) continue; // Delta too large or too small
						newPose.x = newPos;
						break;
					case 3:
						if (newPos - curPose.x > MAX_DELTA.convert(okapi::inch) || newPos - curPose.x < 0) continue; // Delta too large or too small
						if (distConf[3] > distConf[2]) newPose.x = newPos;
						break;
				}
			}
		} else if (theta >= M_PI * 5 / 12.0 && theta <= M_PI * 7 / 12.0) { // Facing east wall; 75-105 deg
			if (east.isInsideSegment(east.getIntersection(dist))) {
				if (i < 2) distToWall *= std::abs(std::sin(theta));
				else distToWall *= std::abs(std::cos(theta));

				if (i == 0 || i == 2) newPos = fieldWidth - distToWall;

				switch (i) {
					case 0:
						if (newPos - curPose.x > MAX_DELTA.convert(okapi::inch) || newPos - curPose.x < 0) continue; // Delta too large or too small
						newPose.x = newPos;
						break;
					case 1:
						if (newPos - curPose.x > MAX_DELTA.convert(okapi::inch) || newPos - curPose.x < 0) continue; // Delta too large or too small
						if (distConf[1] > distConf[0]) newPose.x = newPos;
						break;
					case 2:
						if (newPos - curPose.y > MAX_DELTA.convert(okapi::inch) || newPos - curPose.y < 0) continue; // Delta too large or too small
						newPose.y = newPos;
						break;
					case 3:
						if (newPos - curPose.y > MAX_DELTA.convert(okapi::inch) || newPos - curPose.y < 0) continue; // Delta too large or too small
						if (distConf[3] > distConf[2]) newPose.y = newPos;
						break;
				}
			}
		} else if (theta >= M_PI * 11 / 12.0 && theta <= M_PI * 13 / 12.0) { // Facing south wall; 165-195 deg
			if (south.isInsideSegment(south.getIntersection(dist))) {
				if (i < 2) distToWall *= std::abs(std::cos(theta));
				else distToWall *= std::abs(std::sin(theta));

				if (i == 1 || i == 2) newPos = fieldWidth - distToWall;

				switch (i) {
					case 0:
						if (newPos - curPose.y > MAX_DELTA.convert(okapi::inch) || newPos - curPose.y < 0) continue; // Delta too large or too small
						newPose.y = newPos;
						break;
					case 1:
						if (newPos - curPose.y > MAX_DELTA.convert(okapi::inch) || newPos - curPose.y < 0) continue; // Delta too large or too small
						if (distConf[1] > distConf[0]) newPose.y = newPos;
						break;
					case 2:
						if (newPos - curPose.x > MAX_DELTA.convert(okapi::inch) || newPos - curPose.x < 0) continue; // Delta too large or too small
						newPose.x = newPos;
						break;
					case 3:
						if (newPos - curPose.x > MAX_DELTA.convert(okapi::inch) || newPos - curPose.x < 0) continue; // Delta too large or too small
						if (distConf[3] > distConf[2]) newPose.x = newPos;
						break;
				}
			}
		} else if (theta >= M_PI * 17 / 12.0 && theta <= M_PI * 19 / 12.0) { // Facing west wall; 255-285 deg
			if (west.isInsideSegment(west.getIntersection(dist))) {
				if (i < 2) distToWall *= std::abs(std::sin(theta));
				else distToWall *= std::abs(std::cos(theta));

				if (i == 1 || i == 3) newPos = fieldWidth - distToWall;

				switch (i) {
					case 0:
						if (newPos - curPose.x > MAX_DELTA.convert(okapi::inch) || newPos - curPose.x < 0) continue; // Delta too large or too small
						newPose.x = newPos;
						break;
					case 1:
						if (newPos - curPose.x > MAX_DELTA.convert(okapi::inch) || newPos - curPose.x < 0) continue; // Delta too large or too small
						if (distConf[1] > distConf[0]) newPose.x = newPos;
						break;
					case 2:
						if (newPos - curPose.y > MAX_DELTA.convert(okapi::inch) || newPos - curPose.y < 0) continue; // Delta too large or too small
						newPose.y = newPos;
						break;
					case 3:
						if (newPos - curPose.y > MAX_DELTA.convert(okapi::inch) || newPos - curPose.y < 0) continue; // Delta too large or too small
						if (distConf[3] > distConf[2]) newPose.y = newPos;
						break;
				}
			}
		} else {
			std::cerr << "Invalid theta: " << theta << "\n";
			return;
		}
	}

	if (std::isnan(newPose.x) || std::isnan(newPose.y)) return;
	update(newPose);
	*/
}
void Odometry::update(Pose pose) {
	if (!poseMutex.take(50)) {
		std::cerr << "[Odometry::update] Mutex timout - unable to update pose" << std::endl;
		return;
	}
	this->pose = pose;
	poseMutex.give();
}

std::array<TrackingWheel, 3> Odometry::getEncoders() const {
	return trackingWheels;
}
std::array<DistanceSensor, 4> Odometry::getDistanceSensors() const {
	return distanceSensors;
}
Inertial* Odometry::getInertial() const {
	return inertial;
}

void Odometry::resetSensors() {
	for (int i = 0; i < 3; i++) trackingWheels[i].reset();
	inertial->reset();
}

void Odometry::step(std::vector<double> deltas) {
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

	deltaM -= deltaA * trackingWheels[2].getOffset(); // Original equation: (deltaA / 2pi) * (2pi * middleTrack)
	
	// std::cout << deltaL << " " << deltaR << " " << deltaM << " " << deltaA << "\n";

	// Local offsets
	double localOffsetX = deltaM;
	double localOffsetY = deltaL;
	if (deltaA != 0) {
		localOffsetX = 2 * sin(deltaA / 2.0) * (deltaM / deltaA + trackingWheels[2].getOffset() * 2);
		localOffsetY = 2 * sin(deltaA / 2.0) * (deltaL / deltaA + trackingWheels[1].getOffset() / 2.0);
	}

	// Polar offsets
	double avgA = pose.theta + (deltaA / 2.0);

	double polarR = hypot(localOffsetX, localOffsetY);
	double polarA = atan2(localOffsetY, localOffsetX);

	// Rotating and converting to global offsets
	polarA -= avgA;
	double globalDeltaX = polarR * cos(polarA);
	double globalDeltaY = polarR * sin(polarA);

	if (std::isnan(globalDeltaX)) globalDeltaX = 0;
	if (std::isnan(globalDeltaY)) globalDeltaY = 0;
	if (std::isnan(deltaA)) deltaA = 0;

	double globalX = pose.x + globalDeltaX;
	double globalY = pose.y + globalDeltaY;
	double globalTheta = pose.theta + deltaA;
	if (!poseMutex.take(20)) {
		std::cerr << "[Odometry::step] Mutex timeout - unable to update pose" << std::endl;
		return;
	}
	pose = { globalX * okapi::inch, globalY * okapi::inch, globalTheta * okapi::radian, pros::millis() };
	poseMutex.give();
}

double& Odometry::getDistUpdateCoord(double a, int i, std::array<Pose, 5>& newPose, std::array<int, 4> confs) {
	std::cout << a << " " << i << " ";
	if (std::abs(a - 0) <= 15 || std::abs(a - M_PI) <= 15) {
		switch(i) {
			case 0:
			case 2:
				std::cout << 0.5 * i + 1 << " x\n";
				newPose[0.5 * i + 3].x = confs[i];
				return newPose[0.5 * i + 1].x;
			case 1:
			case 3:
				std::cout << 0.5 * i + 0.5 << " y\n";
				newPose[0.5 * i + 2.5].y = confs[i];
				return newPose[0.5 * i + 0.5].y;
		}
	} else {
		switch(i) {
			case 0:
			case 2:
				std::cout << 0.5 * i + 1 << " y\n";
				newPose[0.5 * i + 3].y = confs[i];
				return newPose[0.5 * i + 1].y;
			case 1:
			case 3:
				std::cout << 0.5 * i + 0.5 << " x\n";
				newPose[0.5 * i + 2.5].x = confs[i];
				return newPose[0.5 * i + 0.5].x;
		}
	}
	return newPose[0].x;
}