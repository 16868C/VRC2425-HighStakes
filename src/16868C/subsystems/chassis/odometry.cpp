#include "16868C/subsystems/chassis/odometry.hpp"
#include "16868C/util/math.hpp"
#include <limits>

using namespace lib16868C;
using namespace okapi::literals;

/** Encoders **/
std::array<double, 3> Encoders::getTicks() const {
	std::array<double, 3> ticks;
	ticks.fill(std::numeric_limits<double>::quiet_NaN());
	if (left != nullptr) ticks[0] = left->get();
	if (right != nullptr) ticks[1] = right->get();
	if (middle != nullptr) ticks[2] = middle->get();
	return ticks;
}
std::array<int, 3> Encoders::getTPR() const {
	std::array<int, 3> tpr;
	tpr.fill(std::numeric_limits<int>::quiet_NaN());
	if (left) tpr[0] = left->getTPR();
	if (right) tpr[1] = left->getTPR();
	if (middle) tpr[2] = left->getTPR();
	return tpr;
}

void Encoders::reset() {
	if (left) left->resetZero();
	if (right) right->resetZero();
	if (middle) middle->resetZero();
}

/** Distances **/
std::array<double, 4> DistanceSnsrs::getDists() const {
	std::array<double, 4> dists;
	dists.fill(std::numeric_limits<double>::quiet_NaN());
	if (front) dists[0] = Util::mToIn(front->get() / 1000.0);
	if (right) dists[1] = Util::mToIn(right->get() / 1000.0);
	if (back) dists[2] = Util::mToIn(back->get() / 1000.0);
	if (left) dists[3] = Util::mToIn(left->get() / 1000.0);
	return dists;
}
std::array<int, 4> DistanceSnsrs::getConfidences() const {
	std::array<int, 4> conf;
	conf.fill(std::numeric_limits<int>::quiet_NaN());
	if (front) conf[0] = front->getConfidence();
	if (right) conf[1] = right->getConfidence();
	if (back) conf[2] = back->getConfidence();
	if (left) conf[3] = left->getConfidence();
	return conf;
}

/** EncoderScales **/
EncoderScales::EncoderScales() {}
EncoderScales::EncoderScales(okapi::QLength leftDiam, double leftGearRatio, okapi::QLength rightDiam, double rightGearRatio, okapi::QLength wheelTrack, okapi::QLength middleDiam, okapi::QLength middleTrack)
	: leftDiam(leftDiam.convert(okapi::inch)), leftGearRatio(leftGearRatio), rightDiam(rightDiam.convert(okapi::inch)), rightGearRatio(rightGearRatio), wheelTrack(wheelTrack.convert(okapi::inch)), middleDiam(middleDiam.convert(okapi::inch)), middleTrack(middleTrack.convert(okapi::inch)) {}
EncoderScales::EncoderScales(okapi::QLength leftDiam, double leftGearRatio, okapi::QLength middleDiam, okapi::QLength middleTrack)
	: leftDiam(leftDiam.convert(okapi::inch)), leftGearRatio(leftGearRatio), middleDiam(middleDiam.convert(okapi::inch)), middleTrack(middleTrack.convert(okapi::inch)) {}

double EncoderScales::getDiam(int index) {
	switch(index) {
		case 0: return leftDiam;
		case 1: return rightDiam;
		case 2: return middleDiam;
		default: return 0;
	}
}
double EncoderScales::getTrack(int index) {
	switch(index) {
		case 0: return wheelTrack;
		case 1: return wheelTrack;
		case 2: return middleTrack;
		default: return 0;
	}
}

/** Distance Scales**/
DistanceScales::DistanceScales() {}
DistanceScales::DistanceScales(okapi::QLength frontDist, okapi::QLength backDist, okapi::QLength leftDist, okapi::QLength rightDist)
	: frontDist(frontDist.convert(okapi::inch)), backDist(backDist.convert(okapi::inch)), leftDist(leftDist.convert(okapi::inch)), rightDist(rightDist.convert(okapi::inch)) {}

double DistanceScales::getDist(int index) {
	switch(index) {
		case 0: return frontDist;
		case 1: return rightDist;
		case 2: return backDist;
		case 3: return leftDist;
		default: return 0;
	}
}

/** Odometry **/
void Odometry::odomManager(void* param) {
	std::vector<double> prev(4, 0);

	Odometry* odom = static_cast<Odometry*>(param);

	uint32_t time = pros::millis();
	while (true) {
		std::array<double, 3> ticks = odom->encs.getTicks();
		if (std::isnan(ticks[0]) && std::isnan(ticks[1]) && std::isnan(ticks[2])) continue;

		std::vector<double> curr;
		// Distance and theta
		curr = odom->ticksToDist(ticks, odom->encs.getTPR());
		if (odom->encs.right) curr.push_back((curr[1] - curr[0]) / odom->encScales.wheelTrack);
		else curr.push_back(odom->inertial->get_rotation(AngleUnit::RAD));

		std::vector<double> deltas;
		for (int i = 0; i < 4; i++) deltas.push_back(curr[i] - prev[i]);
		odom->step(deltas);

		Pose pose = odom->getPose();
		pros::lcd::print(0, "X: %.2f, Y: %.2f", pose.x, pose.y);
		pros::lcd::print(1, "Deg: %.2f, Rad: %.2f", Util::radToDeg(pose.theta), pose.theta);

		Encoders encs = odom->encs;
		DistanceSnsrs dists = odom->dists;
		if (encs.right) pros::lcd::print(2, "Left: %.2f, Right: %.2f", encs.left->get(), encs.right->get());
		else pros::lcd::print(2, "Forward: %.2f", encs.left->get());
		if (odom->inertial) pros::lcd::print(3, "Middle: %.2f, Theta: %.2f", encs.middle->get(), odom->inertial->get_rotation(AngleUnit::DEG));
		else pros::lcd::print(3, "Middle: %.2f", encs.middle->get());

		prev = curr;

		pros::Task::delay_until(&time, 10);
	}
}

Odometry::Odometry() {}
Odometry::Odometry(Encoders encs, DistanceSnsrs dists, std::shared_ptr<Inertial> inertial, EncoderScales encScales, DistanceScales distScales)
	: encs(encs), dists(dists), inertial(inertial), encScales(encScales), distScales(distScales) {}
Odometry::Odometry(std::vector<std::shared_ptr<AbstractEncoder>> encs, EncoderScales encScales) : encScales(encScales) {
	this->encs = {encs[0], encs[1], encs[2]};
}
Odometry::Odometry(std::vector<std::shared_ptr<AbstractEncoder>> encs, Inertial& inertial, EncoderScales encScales) : encScales(encScales) {
	this->encs = {encs[0], nullptr, encs[1]};
	this->inertial = std::make_shared<Inertial>(inertial);
};
Odometry::Odometry(std::vector<std::shared_ptr<okapi::DistanceSensor>> dists, Inertial& inertial, DistanceScales distScales) : distScales(distScales) {
	this->dists = {dists[0], dists[1], dists[2], dists[3]};
	this->inertial = std::make_shared<Inertial>(inertial);
}
Odometry::Odometry(DistanceSnsrs dists, Inertial& inertial, DistanceScales distScales) : dists(dists), distScales(distScales) {
	this->inertial = std::make_shared<Inertial>(inertial);
}
Odometry::Odometry(Odometry& odom) {
	encs = odom.encs;
	dists = odom.dists;
	inertial = odom.inertial;
	encScales = odom.encScales;
	distScales = odom.distScales;
}

void Odometry::init() {
	// Resetting sensors
	pros::delay(100); // Just in case the sensors have not been initialized yet
	encs.reset();
	if (inertial) inertial->calibrate();

	// Resetting pose
	pose = { 0_in, 0_in, 0_rad, 0 };

	// Starting task
	odomTask = pros::c::task_create(odomManager, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odometry");
}
void Odometry::init(Pose pose) {
	// Resetting sensors
	pros::delay(100); // Just in case the sensors have not been initialized yet
	encs.reset();

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
	std::array<double, 4> distReads = dists.getDists();
	std::array<int, 4> distConf = dists.getConfidences();
	std::array<bool, 4> useSnsr {front, right, back, left};
	for (int i = 0; i < 4; i++)
		std::cout << i << ": (" << distReads[i] << ", " << distConf[i] << ") ";
	std::cout << "\n";

	for (int i = 0; i < 4; i++) {
		if (std::isnan(distReads[i]) || !useSnsr[i]) continue; // Not using the sensor
		if (distReads[i] > FIELD_WIDTH.convert(okapi::inch) || distReads[i] < 0) continue; // Read error
		if (distConf[i] < MIN_CONFIDENCE) continue; // Confidence too low

		double distTheta = theta + i * M_PI_2;
		Line dist(std::tan(distTheta), curPose.pos);
		// std::cout << i << " " << dist.toStr() << "\n";
		double distToWall = distReads[i] + distScales.getDist(i), newPos = distToWall;
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

Encoders Odometry::getEncoders() const {
	return encs;
}
EncoderScales Odometry::getEncoderScales() const {
	return encScales;
}
std::shared_ptr<Inertial> Odometry::getInertial() const {
	return inertial;
}
DistanceSnsrs Odometry::getDistanceSensors() const {
	return dists;
}
DistanceScales Odometry::getDistanceScales() const {
	return distScales;
}

void Odometry::resetSensors() {
	encs.reset();
	inertial->reset();
}

void Odometry::step(std::vector<double> deltas) {
	for (double d : deltas) {
		if (std::abs(d) > MAX_DELTA.convert(okapi::inch)) {
			std::cerr << "Odometry delta too large: " << d << "\n";
			return;
		}
	}

	// Delta Distances
	double deltaL = deltas[0];
	double deltaR = deltas[1];
	double deltaM = deltas[2];
	double deltaA = deltas[3];

	deltaM -= deltaA * encScales.middleTrack; // Original equation: (deltaA / 2pi) * (2pi * middleTrack)
	
	// std::cout << deltaL << " " << deltaR << " " << deltaM << " " << deltaA << "\n";

	// Local offsets
	double localOffsetX = deltaM;
	double localOffsetY = deltaL;
	if (deltaA != 0) {
		localOffsetX = 2 * sin(deltaA / 2.0) * (deltaM / deltaA + encScales.middleTrack * 2);
		localOffsetY = 2 * sin(deltaA / 2.0) * (deltaL / deltaA + encScales.wheelTrack / 2.0);
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

std::vector<double> Odometry::ticksToDist(std::array<double, 3> ticks, std::array<int, 3> tpr)  {
	std::vector<double> dists;
	for (int i = 0; i < ticks.size(); i++) {
		if (std::isnan(ticks[i]) || std::isnan(tpr[i])) continue;
		dists[i] = ticks[i] / tpr[i] * encScales.getDiam(i) * M_PI;
	}
	return dists;
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