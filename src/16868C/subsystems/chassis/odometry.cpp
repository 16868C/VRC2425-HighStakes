#include "16868C/subsystems/chassis/odometry.hpp"
#include <limits>

using namespace lib16868C;
using namespace okapi::literals;

/** Encoders **/
std::vector<double> Encoders::getTicks() const {
	std::vector<double> ticks(3, std::numeric_limits<double>::quiet_NaN());
	if (left) ticks[0] = left->get();
	if (right) ticks[1] = right->get();
	if (middle) ticks[2] = middle->get();
	return ticks;
}
std::vector<int> Encoders::getTPR() const {
	std::vector<int> tpr(3, std::numeric_limits<int>::quiet_NaN());
	if (left) tpr.push_back(left->getTPR());
	if (right) tpr.push_back(right->getTPR());
	if (middle) tpr.push_back(middle->getTPR());
	return tpr;
}

void Encoders::reset() {
	if (left) left->resetZero();
	if (right) right->resetZero();
	if (middle) middle->resetZero();
}

/** Distances **/
std::vector<double> DistanceSnsrs::getDists() const {
	std::vector<double> dists(4, std::numeric_limits<double>::quiet_NaN());
	if (front) dists[0] = Util::mToIn(front->get() / 1000.0);
	if (back) dists[1] = Util::mToIn(back->get() / 1000.0);
	if (left) dists[2] = Util::mToIn(left->get() / 1000.0);
	if (right) dists[3] = Util::mToIn(right->get() / 1000.0);
	return dists;
}
std::vector<int> DistanceSnsrs::getConfidences() const {
	std::vector<int> conf(4, std::numeric_limits<int>::quiet_NaN());
	if (front) conf[0] = front->getConfidence();
	if (back) conf[1] = back->getConfidence();
	if (left) conf[2] = left->getConfidence();
	if (right) conf[3] = right->getConfidence();
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
double DistanceScales::getDist(int index) {
	switch(index) {
		case 0: return frontDist;
		case 1: return backDist;
		case 2: return leftDist;
		case 3: return rightDist;
		default: return 0;
	}
}

/** Odometry **/
void Odometry::odomManager(void* param) {
	std::vector<double> prev(4, 0);

	Odometry* odom = static_cast<Odometry*>(param);

	uint32_t time = pros::millis();
	while (true) {
		std::vector<double> curr;
		// Distance and theta
		curr = odom->ticksToDist(odom->encs.getTicks(), odom->encs.getTPR());
		if (odom->encs.right) curr.push_back((curr[1] - curr[0]) / odom->encScales.wheelTrack);
		else curr.push_back(Util::degToRad(odom->inertial->get_rotation()));

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
		if (odom->inertial) pros::lcd::print(3, "Middle: %.2f, Theta: %.2f", encs.middle->get(), odom->inertial->get_rotation());
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
Odometry::Odometry(std::vector<std::shared_ptr<AbstractEncoder>> encs, Inertial inertial, EncoderScales encScales) : encScales(encScales) {
	this->encs = {encs[0], nullptr, encs[1]};
	this->inertial = std::make_shared<Inertial>(inertial);
};
Odometry::Odometry(std::vector<std::shared_ptr<okapi::DistanceSensor>> dists, Inertial inertial, DistanceScales distScales) : distScales(distScales) {
	// this->dists.front = std::make_shared<okapi::DistanceSensor>(dists[0]);
	// this->dists.back = std::make_shared<okapi::DistanceSensor>(dists[1]);
	// this->dists.left = std::make_shared<okapi::DistanceSensor>(dists[2]);
	// this->dists.right = std::make_shared<okapi::DistanceSensor>(dists[3]);
	this->dists.front = dists[0];
	this->dists.back = dists[1];
	this->dists.left = dists[2];
	this->dists.right = dists[3];
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

void Odometry::update() {
	Pose newPose;

	Pose curPose = getPose();
	double theta = ReduceAngle::radPi2(curPose.theta);
	if (std::abs(theta) > M_PI / 12.0 && std::abs(theta) < M_PI * 5 / 12.0) return; // Not perpendicular to wall
	theta = ReduceAngle::rad2Pi(curPose.theta);
	std::vector<double> distReads = dists.getDists();
	std::vector<int> distConf = dists.getConfidences();

	for (int i = 0; i < 4; i++) {
		if (std::isnan(distReads[i])) continue; // Not using the sensor
		if (distReads[i] > FIELD_WIDTH.convert(okapi::inch) || distReads[i] < 0) continue; // Read error
		if (distConf[i] < MIN_CONFIDENCE) continue; // Not confident enough

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

std::vector<double> Odometry::ticksToDist(std::vector<double> ticks, std::vector<int> tpr)  {
	if (ticks.size() != tpr.size()) { std::cerr << "ticksToDist: ticks and tpr must be the same size\n"; return std::vector<double>(ticks.size(), 0); }

	std::vector<double> dists(3);
	for (int i = 0; i < ticks.size(); i++) {
		if (std::isnan(ticks[i]) || std::isnan(tpr[i])) continue;
		dists[i] = ticks[i] / tpr[i] * encScales.getDiam(i) * M_PI;
	}
	return dists;
}