#include "16868C/subsystems/chassis/odometry.hpp"

using namespace lib16868C;
using namespace okapi::literals;

/** OdomSensors **/
void OdomSensors::reset() {
	if (left) left->resetZero();
	if (right) right->resetZero();
	if (middle) middle->resetZero();
	if (inertial) inertial->reset(true);
};
std::vector<double> OdomSensors::getTicks() {
	if (right) return {left->get(), right->get(), middle->get()};
	else return {left->get(), 0, middle->get()};
}
std::vector<int> OdomSensors::getTPR() {
	if (right) return {left->getTPR(), right->getTPR(), middle->getTPR()};
	else return {left->getTPR(), 0x3f, middle->getTPR()};
}

/** EncoderScales **/
EncoderScales::EncoderScales(okapi::QLength leftDiam, okapi::QLength rightDiam, okapi::QLength wheelTrack, okapi::QLength middleDiam, okapi::QLength middleTrack)
	: leftDiam(leftDiam.convert(okapi::inch)), rightDiam(rightDiam.convert(okapi::inch)), wheelTrack(wheelTrack.convert(okapi::inch)), middleDiam(middleDiam.convert(okapi::inch)), middleTrack(middleTrack.convert(okapi::inch)) {}
EncoderScales::EncoderScales(okapi::QLength leftDiam, okapi::QLength middleDiam, okapi::QLength middleTrack)
	: leftDiam(leftDiam.convert(okapi::inch)), middleDiam(middleDiam.convert(okapi::inch)), middleTrack(middleTrack.convert(okapi::inch)) {}

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

/** Odometry **/
void Odometry::odomManager(void* param) {
	std::vector<double> prev(4, 0);

	Odometry* odom = static_cast<Odometry*>(param);

	uint32_t time = pros::millis();
	while (true) {
		std::vector<double> curr;
		// Distance and theta
		curr = odom->ticksToDist(odom->snsrs.getTicks(), odom->snsrs.getTPR());
		if (odom->snsrs.right) curr.push_back((curr[1] - curr[0]) / odom->encScales.wheelTrack);
		else curr.push_back(Util::degToRad(odom->snsrs.inertial->get_rotation()));

		std::vector<double> deltas;
		for (int i = 0; i < 4; i++) deltas.push_back(curr[i] - prev[i]);
		odom->step(deltas);

		Pose pose = odom->getPose();
		pros::lcd::print(0, "X: %.2f, Y: %.2f", pose.x, pose.y);
		pros::lcd::print(1, "Deg: %.2f, Rad: %.2f", Util::radToDeg(pose.theta), pose.theta);

		OdomSensors snsrs = odom->getSensors();
		if (snsrs.right) pros::lcd::print(2, "Left: %.2f, Right: %.2f", snsrs.left->get(), snsrs.right->get());
		else pros::lcd::print(2, "Forward: %.2f", snsrs.left->get());
		if (snsrs.inertial) pros::lcd::print(3, "Middle: %.2f, Theta: %.2f", snsrs.middle->get(), snsrs.inertial->get_rotation());
		else pros::lcd::print(3, "Middle: %.2f", snsrs.middle->get());

		prev = curr;

		pros::Task::delay_until(&time, 10);
	}
}

Odometry::Odometry(OdomSensors snsrs, EncoderScales encScales) : snsrs(snsrs), encScales(encScales) {}
Odometry::Odometry(std::vector<Rotation> encs, EncoderScales encScales) : encScales(encScales) {
	snsrs.left = std::make_shared<Rotation>(encs[0]);
	snsrs.right = std::make_shared<Rotation>(encs[1]);
	snsrs.middle = std::make_shared<Rotation>(encs[2]);
}
Odometry::Odometry(std::vector<Rotation> encs, pros::Imu inertial, EncoderScales encScales) : encScales(encScales) {
	snsrs.left = std::make_shared<Rotation>(encs[0]);
	snsrs.middle = std::make_shared<Rotation>(encs[1]);
	snsrs.inertial = std::make_shared<pros::Imu>(inertial);
};
Odometry::Odometry(std::vector<OpticalEncoder> encs, EncoderScales encScales) : encScales(encScales) {
	snsrs.left = std::make_shared<OpticalEncoder>(encs[0]);
	snsrs.right = std::make_shared<OpticalEncoder>(encs[1]);
	snsrs.middle = std::make_shared<OpticalEncoder>(encs[2]);
}
Odometry::Odometry(std::vector<OpticalEncoder> encs, pros::Imu inertial, EncoderScales encScales) : encScales(encScales) {
	snsrs.left = std::make_shared<OpticalEncoder>(encs[0]);
	snsrs.middle = std::make_shared<OpticalEncoder>(encs[1]);
	snsrs.inertial = std::make_shared<pros::Imu>(inertial);
}

void Odometry::init() {
	// Resetting sensors
	pros::delay(100); // Just in case the sensors have not been initialized yet
	snsrs.reset();

	// Resetting pose
	pose = { 0_in, 0_in, 0_rad, 0 };

	// Starting task
	odomTask = pros::c::task_create(odomManager, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odometry");
}
void Odometry::init(Pose pose) {
	// Resetting sensors
	snsrs.reset();

	// Resetting pose
	this->pose = pose;

	// Starting task
	odomTask = pros::c::task_create(odomManager, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odometry");
}

Pose Odometry::getPose() {
	return pose;
}

OdomSensors Odometry::getSensors() {
	return snsrs;
}
void Odometry::resetSensors() {
	snsrs.reset();
}

EncoderScales Odometry::getEncoderScales() {
	return encScales;
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
	pose = { globalX * okapi::inch, globalY * okapi::inch, globalTheta * okapi::radian, pros::millis() };
}

std::vector<double> Odometry::ticksToDist(std::vector<double> ticks, std::vector<int> tpr)  {
	if (ticks.size() != tpr.size()) { std::cerr << "ticksToDist: ticks and tpr must be the same size\n"; return std::vector<double>(ticks.size(), 0); }

	std::vector<double> dists;
	for (int i = 0; i < ticks.size(); i++)
		dists.push_back(ticks[i] / tpr[i] * encScales.getDiam(i) * M_PI);
	return dists;
}