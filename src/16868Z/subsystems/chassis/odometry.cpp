#include "16868Z/subsystems/chassis/odometry.hpp"

using namespace lib16868Z;

void Odometry::odomManager(void* param) {
	EncoderVals prev = {0, 0, 0, 0, EncoderValsType::TICKS};

	uint32_t time = pros::millis();
	while (true) {
		EncoderVals curr = odomSingleton->getEncoderTicks();

		odomSingleton->step(curr - prev);

		Pose pose = odomSingleton->getPose();
		pros::lcd::print(0, "X: %.2f, Y: %.2f", pose.x, pose.y);
		pros::lcd::print(1, "Deg: %.2f, Rad: %.2f", Util::radToDeg(pose.theta), pose.theta);

		pros::lcd::print(2, "Left: %.2f, Right: %.2f", curr.left, curr.right);
		pros::lcd::print(3, "Rear: %.2f", curr.rear);

		pros::Task::delay_until(&time, 10);
	}
}

std::shared_ptr<Odometry> Odometry::odomSingleton = nullptr;

std::shared_ptr<Odometry> Odometry::getOdometry() {
	return odomSingleton;
}
std::shared_ptr<Odometry> Odometry::getOdometry(Encoders encs, EncoderScales wheelDiamScales, EncoderScales wheelTrackScales) {
	if (odomSingleton) return odomSingleton;

	if (wheelDiamScales.type != EncoderValsType::SCALES_WHEEL_DIAM) return nullptr;
	if (wheelTrackScales.type != EncoderValsType::SCALES_WHEEL_TRACK) return nullptr;
	Odometry odom(encs, wheelDiamScales, wheelTrackScales);
	return odomSingleton = std::make_shared<Odometry>(odom);
}
std::shared_ptr<Odometry> Odometry::getOdometry(Encoders encs, EncoderScales wheelDiamScales, EncoderScales wheelTrackScales, std::shared_ptr<pros::Imu> inertial) {
	if (odomSingleton) return odomSingleton;

	if (wheelDiamScales.type != EncoderValsType::SCALES_WHEEL_DIAM) return nullptr;
	if (wheelTrackScales.type != EncoderValsType::SCALES_WHEEL_TRACK) return nullptr;
	Odometry odom(encs, wheelDiamScales, wheelTrackScales, inertial);
	return odomSingleton = std::make_shared<Odometry>(odom);
}

Odometry::Odometry(Encoders encs, EncoderScales wheelDiamScales, EncoderScales wheelTrackScales) 
				: encs(encs), wheelDiamScales(wheelDiamScales), wheelTrackScales(wheelTrackScales) {}
Odometry::Odometry(Encoders encs, EncoderScales wheelDiamScales, EncoderScales wheelTrackScales, std::shared_ptr<pros::Imu> inertial) 
				: encs(encs), wheelDiamScales(wheelDiamScales), wheelTrackScales(wheelTrackScales), inertial(inertial) {
	useInertial = true;
}

void Odometry::init() {
	// Resetting sensors
	encs.reset();
	if (useInertial) inertial->reset();

	// Resetting pose
	pose = { 0, 0, 0, 0 };

	// Starting task
	odomTask = pros::c::task_create(odomManager, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odometry");
}
void Odometry::init(Pose pose) {
	// Resetting sensors
	encs.reset();
	if (useInertial) inertial->reset();

	// Resetting pose
	this->pose = pose;

	// Starting task
	odomTask = pros::c::task_create(odomManager, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odometry");
}

Pose Odometry::getPose() {
	return pose;
}

Encoders Odometry::getEncoders() {
	return encs;
}
void Odometry::resetEncoders() {
	encs.reset();
}
EncoderTicks Odometry::getEncoderTicks() {
	return encs.getTicks();
}

EncoderScales Odometry::getWheelDiamScales() {
	return wheelDiamScales;
}
EncoderScales Odometry::getWheelTrackScales() {
	return wheelTrackScales;
}

bool Odometry::isUsingInertial() {
	return useInertial;
}

EncoderScales Odometry::calibWheelDiam(double actualDist) {
	EncoderTicks ticksTravelled = encs.getTicks();
	EncoderVals distTravelled = ticksTravelled.toDistance(wheelDiamScales, encs.tpr);
	EncoderVals ratios = distTravelled / actualDist;
	EncoderVals newScales = wheelDiamScales * ratios;

	std::cout << "Ticks travelled: " << ticksTravelled.left << ", " << ticksTravelled.right << ", " << ticksTravelled.rear << "\n";
	std::cout << "Distance travelled: " << distTravelled.left << ", " << distTravelled.right << ", " << distTravelled.rear << "\n";
	std::cout << "Ratios: " << ratios.left << ", " << ratios.right << ", " << ratios.rear << "\n";
	std::cout << "Calibrated wheel diameters: " << newScales.left << ", " << newScales.right << ", " << newScales.rear << "\n";
	return newScales;
}

EncoderScales Odometry::calibWheelTrack(double actualAng) {
	EncoderTicks ticksTravelled = encs.getTicks();
	EncoderVals distTravelled = ticksTravelled.toDistance(wheelDiamScales, encs.tpr);
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

void Odometry::step(EncoderVals deltaTicks) {
	if (deltaTicks.type != EncoderValsType::TICKS) return;
	if (std::abs(deltaTicks.left) > MAX_TICKS || std::abs(deltaTicks.right) > MAX_TICKS || std::abs(deltaTicks.rear) > MAX_TICKS) {
		std::cerr << "Odometry: Encoder delta too large: " << deltaTicks.left << ", " << deltaTicks.right << ", " << deltaTicks.rear << "\n";
		return;
	}

	// Chassis scales
	const double leftCircumference = wheelDiamScales.left * M_PI;
	const double rightCircumference = wheelDiamScales.right * M_PI;
	const double rearCircumference = wheelDiamScales.rear * M_PI;
	const double wheelTrack = wheelTrackScales.left + wheelTrackScales.right;

	// Delta distance and theta
	EncoderVals delta = deltaTicks.toDistance(wheelDiamScales, encs.tpr);
	delta.theta = (delta.right - delta.left) / wheelTrack;
	delta.rear -= delta.theta * wheelTrackScales.rear;

	// Local coordinates
	double localOffsetX = delta.rear;
	double localOffsetY = (delta.left + delta.right) / 2.0;
	if (delta.left != delta.right) {
		localOffsetX = 2 * sin(delta.theta / 2.0) * (delta.rear / delta.theta + wheelTrackScales.rear);
		localOffsetY = 2 * sin(delta.theta / 2.0) * (delta.right / delta.theta + wheelTrackScales.right);
	}

	// Polar coordinates
	double polarR = std::hypot(localOffsetX, localOffsetY);
	double polarA = std::atan2(localOffsetY, localOffsetX);

	// Global coordinates
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