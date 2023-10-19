#include "16868C/subsystems/chassis/odometry.hpp"

using namespace lib16868C;
using namespace okapi::literals;

void Odometry::odomManager(void* param) {
	std::vector<double> prev(4, 0);

	Odometry* odom = static_cast<Odometry*>(param);

	uint32_t time = pros::millis();
	while (true) {
		std::vector<double> curr;
		// Distance and theta
		curr = odom->ticksToDist(odom->snsrs.getTicks(), odom->snsrs.getTPR());
		if (odom->snsrs.right) curr.push_back((curr[1] - curr[0]) / odom->encScales.wheelTrack.convert(okapi::inch));
		else curr.push_back(Util::degToRad(odom->snsrs.inertial->get_rotation()));

		std::vector<double> deltas;
		for (int i = 0; i < 4; i++) deltas.push_back(curr[i] - prev[i]);
		odom->step(deltas);

		Pose pose = odom->getPose();
		pros::lcd::print(0, "X: %.2f, Y: %.2f", pose.x.convert(okapi::inch), pose.y.convert(okapi::inch));
		pros::lcd::print(1, "Deg: %.2f, Rad: %.2f", pose.theta.convert(okapi::degree), pose.theta.convert(okapi::radian));

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
	pose = { 0_in, 0_in, 0_deg, 0 };

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

	deltaM -= deltaA * encScales.middleTrack.convert(okapi::inch); // Original equation: (deltaA / 2pi) * (2pi * middleTrack)
	
	// std::cout << deltaL << " " << deltaR << " " << deltaM << " " << deltaA << "\n";

	// Local offsets
	double localOffsetX = deltaM;
	double localOffsetY = deltaL;
	if (deltaA != 0) {
		localOffsetX = 2 * sin(deltaA / 2.0) * (deltaM / deltaA + encScales.middleTrack.convert(okapi::inch) * 2);
		localOffsetY = 2 * sin(deltaA / 2.0) * (deltaL / deltaA + encScales.wheelTrack.convert(okapi::inch) / 2.0);
	}

	// Polar offsets
	double avgA = pose.theta.convert(okapi::radian) + (deltaA / 2.0);

	double polarR = hypot(localOffsetX, localOffsetY);
	double polarA = atan2(localOffsetY, localOffsetX);

	// Rotating and converting to global offsets
	polarA -= avgA;
	double globalDeltaX = polarR * cos(polarA);
	double globalDeltaY = polarR * sin(polarA);

	if (std::isnan(globalDeltaX)) globalDeltaX = 0;
	if (std::isnan(globalDeltaY)) globalDeltaY = 0;
	if (std::isnan(deltaA)) deltaA = 0;

	double globalX = pose.x.convert(okapi::inch) + globalDeltaX;
	double globalY = pose.y.convert(okapi::inch) + globalDeltaY;
	double globalTheta = pose.theta.convert(okapi::radian) + deltaA;
	pose = {globalX * okapi::inch, globalY * okapi::inch, globalTheta * okapi::radian, pros::millis()};
}