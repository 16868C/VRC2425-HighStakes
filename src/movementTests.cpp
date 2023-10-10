#include "okapi/api.hpp"
#include "robotconfig.hpp"
#include "16868Z/controllers/PIDController.hpp"
#include "16868Z/subsystems/chassis/motionProfiling.hpp"
#include "16868Z/util/util.hpp"
#include <fstream>

using namespace lib16868C;

void moveDistanceStraight(double dist, double maxRPM) {
	pros::Task vel = pros::Task([&] {
		okapi::VelMath mtrVelCalc = okapi::VelMathFactory().create(900);
		okapi::VelMath encVelCalc = okapi::VelMathFactory().create(360);
		while (true) {
			double motorVel = mtrVelCalc.step((leftDrive.getPosition() + rightDrive.getPosition()) / 2.0).convert(okapi::rpm);
			// double encVel = encVelCalc.step((leftEncoder.get() + rightEncoder.get()) / 2.0).convert(okapi::rpm);
			// std::cout << motorVel << " " << encVel * (2/6.0) << " " << inertial.get_rotation() << "\n";
			pros::delay(50);
		}
	});

	double currDist = 0;
	while (dist - currDist > 0) {
		// double avgTicks = std::abs(leftEncoder.get() + rightEncoder.get()) / 2.0;
		// currDist = avgTicks / 360.0 * WHEEL_DIAMETER * M_PI;
		leftDrive.moveVoltage(maxRPM / 200 * 12000);
		rightDrive.moveVoltage(maxRPM / 200 * 12000);
		pros::delay(20);
	}
	std::cout << currDist << "\n";
	
	leftDrive.moveVelocity(0);
	rightDrive.moveVelocity(0);
	leftDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	rightDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	std::cout << static_cast<int>(leftDrive.getBrakeMode()) << "\n";
	pros::delay(10000000);
}

void moveDistanceABS(double dist, double maxRPM, double decelDist, PIDGains leftPid, PIDGains rightPid, double turnRPM, PIDGains headingPid, double minPower) {
	leftDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	rightDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

	double targetHeading = inertial.get_rotation();

	PIDController leftPID(leftPid, 1.0, 0.0);
	PIDController rightPID(rightPid, 1.0, 0.0);
	PIDController headingPID(headingPid, 1.0, -1.0, 25, 10, true);

	double leftVel = minPower, prevLeftVel = minPower, rightVel = minPower, prevRightVel = minPower;
	// double leftDist = leftEncoder.get() / 360.0 * 2.75 * M_PI;
	// double rightDist = rightEncoder.get() / 360.0 * 2.75 * M_PI;
	double leftDist = 0, rightDist = 0;
	double leftSgnl = 0, rightSgnl = 0;
	double headingSgnl = 0;

	uint st = pros::millis();
	double leftMtrVel = 0, rightMtrVel = 0;
	pros::Task vel = pros::Task([&] {
		std::ofstream data("/usd/data.csv");
		int lastSaved = 0;

		std::cout << "LeftMtrVel RightMtrVel EncRPM LeftVel RightVel LeftEncDist RightEncDist LeftPIDSgnl RightPIDSgnl Heading\n";
		data << "LeftMtrVel,RightMtrVel,EncRPM,LeftVel,RightVel,LeftEncDist,RightEncDist,LeftPIDSgnl,RightPIDSgnl,Heading\n";
		okapi::VelMath leftMtrVelCalc = okapi::VelMathFactory().create(300);
		okapi::VelMath rightMtrVelCalc = okapi::VelMathFactory().create(300);
		okapi::VelMath encVelCalc = okapi::VelMathFactory().create(360);
		while (true) {
			leftMtrVel = leftMtrVelCalc.step(leftDrive.getPosition()).convert(okapi::rpm);
			rightMtrVel = rightMtrVelCalc.step(rightDrive.getPosition()).convert(okapi::rpm);
			// double encVel = encVelCalc.step((leftEncoder.get() + rightEncoder.get()) / 2.0).convert(okapi::rpm);
			// double encInps = encVel / 60.0 * (M_PI * 2.75);
			// double encRPM = encInps * 60 / (M_PI * 4.0) / (4/6.0);
			// std::cout << leftMtrVel << " " << rightMtrVel << " " << encRPM << " " << leftVel << " " << rightVel << " " << leftDist << " " << rightDist << " " << leftSgnl << " " << rightSgnl << " " << inertial.get_rotation() << "\n";
			// data << leftMtrVel << "," << rightMtrVel << "," << encRPM << "," << leftVel << "," << rightVel << "," << leftDist << "," << rightDist << "," << leftSgnl << "," << rightSgnl << "," << inertial.get_rotation() << "\n";
			lastSaved++;
			if (lastSaved >= 20) { data.close(); data.open("/usd/data.csv", std::ios::trunc); }

			// if (leftMtrVel == 0 && rightMtrVel == 0) break;
			pros::delay(50);
		}
	});

	pros::Task left = pros::Task([&] {
		while (true) {
			if (dist - leftDist < 0.5) leftDrive.moveVelocity(0);
			else if (std::abs(dist - leftDist) < decelDist) {
				if (std::abs(leftMtrVel) <= minPower) {
					leftDrive.moveVoltage((minPower * Util::sgn(dist - leftDist)) / 600.0 * 12000);
					pros::delay(20);
				} else {
					leftDrive.moveVelocity(0);
					pros::delay(20 * (1 - leftSgnl));
					leftDrive.moveVoltage(0);
					pros::delay(20 * leftSgnl);
				}
				// leftDrive.moveVoltage((leftVel + (turnRPM * headingSgnl)) / 600.0 * 12000);
				// pros::delay(20);
			} else {
				leftDrive.moveVoltage((leftVel + (turnRPM * headingSgnl)) / 600.0 * 12000);
				pros::delay(20);
			}
		}
	});
	pros::Task right = pros::Task([&] {
		while (true) {
			if (dist - rightDist < 0.5) rightDrive.moveVelocity(0);
			else if (std::abs(dist - rightDist) < decelDist) {
				if (std::abs(rightMtrVel) <= minPower) {
					rightDrive.moveVoltage((minPower * Util::sgn(dist - rightDist)) / 600.0 * 12000);
					pros::delay(20);
				} else {
					rightDrive.moveVelocity(0);
					pros::delay(20 * (1 - rightSgnl));
					rightDrive.moveVoltage(0);
					pros::delay(20 * rightSgnl);
				}
			} else {
				rightDrive.moveVoltage((rightVel - (turnRPM * headingSgnl)) / 600.0 * 12000);
				pros::delay(20);
			}
		}
	});
	while (std::abs(dist - leftDist) >= 0.5 || std::abs(dist - rightDist) >= 0.5 || std::abs(targetHeading - inertial.get_rotation()) >= 1) {
		// leftDist = leftEncoder.get() / 360.0 * 2.75 * M_PI;
		// rightDist = rightEncoder.get() / 360.0 * 2.75 * M_PI;
		// leftDist = leftDrive.getPosition() / 300.0 * 4 * M_PI * (4/6.0);
		// rightDist = rightDrive.getPosition() / 360.0 * 4 * M_PI * (4/6.0);

		headingSgnl = headingPID.calculate(targetHeading, inertial.get_rotation());
		leftSgnl = leftPID.calculate(dist, leftDist);
		rightSgnl = rightPID.calculate(dist, rightDist);

		leftVel = std::min(maxRPM, prevLeftVel * 1.03); prevLeftVel = leftVel;
		rightVel = std::min(maxRPM, prevRightVel * 1.03); prevRightVel = rightVel;

		pros::delay(20);
	}

	left.remove();
	right.remove();

	leftDrive.moveVelocity(0);
	rightDrive.moveVelocity(0);

	// leftDrive.moveAbsolute(ticks, maxRPM);
	// rightDrive.moveAbsolute(ticks, maxRPM);

	// while (leftDrive.getPosition() < ticks) pros::delay(20);
	
	// std::cout << "decel\n";
	// while (motorVel > 0) {
	// 	leftDrive.moveVelocity(0);
	// 	rightDrive.moveVelocity(0);
	// 	pros::delay(5);
	// 	leftDrive.moveAbsolute(ticks, maxRPM);
	// 	rightDrive.moveAbsolute(ticks, maxRPM);
	// 	pros::delay(45);
	// }

	// leftDrive.moveVelocity(0);
	// rightDrive.moveVelocity(0);
	// std::cout << "done\n";

	// double prevVel = 50;
	// double currDist = 0;
	// while (dist - currDist > decel) {
	// 	double avgTicks = std::abs(leftEncoder.get() + rightEncoder.get()) / 2.0;
	// 	currDist = avgTicks / 360.0 * WHEEL_DIAMETER * M_PI;
		
	// 	double vel = std::min(maxRPM, prevVel * 1.02);
	// 	// double vel = maxRPM;
	// 	prevVel = vel;
	// 	leftDrive.moveVoltage(vel / 200 * 12000);
	// 	rightDrive.moveVoltage(vel / 200 * 12000);
	// 	pros::delay(20);
	// }
	// std::cout << currDist << "\n";
	
	// leftDrive.moveVoltage(0);
	// rightDrive.moveVoltage(0);
	// std::cout << "decel\n";
	// int c = 0;
	// while (true) {
	// 	leftDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	// 	rightDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	// 	pros::delay(30);
	// 	leftDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	// 	rightDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	// 	pros::delay(20);
	// 	c++;
	// 	if (leftDrive.getActualVelocity() == 0 && rightDrive.getActualVelocity() == 0)  {
	// 		std::cout << "time: " << pros::millis() - st << "\n";
	// 		break;
	// 	}
	// }
	// std::cout << c << "\n";
	// pros::delay(100000);
}

void moveDistanceABS2(double dist, double maxRPM, double decelDist, PIDGains absPID, double turnRPM, PIDGains headingPID, double minPower) {
	// leftDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	// rightDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	leftDrive.tarePosition();
	rightDrive.tarePosition();
	
	double targetHeading = inertial.get_rotation();
	
	PIDController absController(absPID, 1, 0);
	PIDController headingController(headingPID, 1, -1);

	double currDist = 0;
	double prevVel = minPower / 12000 * 600;

	okapi::VelMath leftMtrVelCalc = okapi::VelMathFactory().create(300);
	okapi::VelMath rightMtrVelCalc = okapi::VelMathFactory().create(300);
	double leftMtrVel = 0, rightMtrVel = 0;

	while (std::abs(dist - currDist) > 1) {
		leftMtrVel = leftMtrVelCalc.step(leftDrive.getPosition()).convert(okapi::rpm);
		rightMtrVel = rightMtrVelCalc.step(rightDrive.getPosition()).convert(okapi::rpm);

		currDist = (leftDrive.getPosition() + rightDrive.getPosition()) / 2.0 / 300.0 * 4.0 * M_PI * (3 / 6.0);
		std::cout << currDist << "\n";
		double absSgnl = absController.calculate(dist, currDist);
		double headingSgnl = headingController.calculate(targetHeading, inertial.get_rotation());

		if (dist - currDist < decelDist) {
			leftDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
			rightDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
			leftDrive.moveVelocity(0);
			rightDrive.moveVelocity(0);
			pros::delay(15 * (1 - absSgnl));
			leftDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			rightDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			leftDrive.moveVoltage(0);
			rightDrive.moveVoltage(0);
			pros::delay(15 * absSgnl);
			// std::cout << leftMtrVel << " " << rightMtrVel << " " << absSgnl << "\n";
		} else {
			double vel = std::min(maxRPM, prevVel * 1.04); prevVel = vel;
			leftDrive.moveVoltage((vel + turnRPM * headingSgnl) / 600 * 12000);
			rightDrive.moveVoltage((vel - turnRPM * headingSgnl) / 600 * 12000);
			pros::delay(15);
		}
	}
	leftDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	rightDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	leftDrive.moveVelocity(0);
	rightDrive.moveVelocity(0);
}

void moveDistancePIDABS(double dist, double maxRPM, double heading, double turnRPM, double accel, PIDGains decelGains, PIDGains headingGains) {
	MotionLimit motionLimit = {maxRPM * 4 * M_PI / 60.0 / (4/6.0), accel};
	MotionProfile profile = MotionProfiling::generateAccel(dist, motionLimit);
	
	double leftVel = 0, rightVel = 0;
	pros::Task vel = pros::Task([&]() {
		okapi::VelMath leftCalc = okapi::VelMathFactory().create(300);
		okapi::VelMath rightCalc = okapi::VelMathFactory().create(300);
		while (true) {
			leftVel = leftCalc.step(leftDrive.getPosition()).convert(okapi::rpm);
			rightVel = rightCalc.step(rightDrive.getPosition()).convert(okapi::rpm);
			pros::delay(20);
		}
	});

	PIDController distPID(decelGains);
	PIDController headingPID(headingGains);

	double currDist = 0;
	int i = 0;
	uint32_t time = pros::millis(), st = pros::millis(), t = pros::millis();
	int dir = Util::sgn(dist);
	while (std::abs(dist) - currDist > 0.1 || heading - inertial.get_rotation() > 1) {
		t = pros::millis();
		double headingCtrl = headingPID.calculate(heading, inertial.get_rotation());
		// double avgTicks = std::abs(leftEncoder.get() + rightEncoder.get()) / 2.0;
		// currDist = avgTicks / 360.0 * 2.75 * M_PI;
		double avgTicks = std::abs(leftDrive.getPosition() + rightDrive.getPosition()) / 2.0;
		currDist = (avgTicks / 300.0) * 4 * M_PI * (4/6.0);
		double distCtrl = distPID.calculate(dist, currDist);

		double accelRPM = (t - st <= profile.accelTime * 1000 ? profile[i++].velocity * 60 / (M_PI * 4) * (4/6.0) : maxRPM);
		double decelRPM = maxRPM * distCtrl;
		double vel = std::min(maxRPM, std::min(accelRPM, decelRPM));
		if (vel == decelRPM) {
			if (leftVel > vel) leftDrive.moveVelocity(0);
			if (rightVel > vel) rightDrive.moveVelocity(0);
			pros::delay(20);
		}
		leftDrive.moveVoltage(((dir * vel) + (turnRPM * headingCtrl)) / 600.0 * 12000);
		rightDrive.moveVoltage(((dir * vel) - (turnRPM * headingCtrl)) / 600.0 * 12000);
		// leftDrive.moveVelocity(vel);
		// rightDrive.moveVelocity(vel);
		std::cout << currDist << " " << leftVel << " " << rightVel << " " << vel << "\n";

		pros::Task::delay_until(&time, 20);
	}

	leftDrive.moveVelocity(0);
	rightDrive.moveVelocity(0);
}

void moveDistancePID(double dist, double maxRPM, double heading, double turnRPM, double accel, PIDGains decelGains, PIDGains headingGains) {
	MotionLimit motionLimit = {maxRPM * 4 * M_PI / 60.0 / (4/6.0), accel};
	MotionProfile profile = MotionProfiling::generateAccel(dist, motionLimit);
	
	double leftVel = 0, rightVel = 0;
	pros::Task vel = pros::Task([&]() {
		okapi::VelMath leftCalc = okapi::VelMathFactory().create(300);
		okapi::VelMath rightCalc = okapi::VelMathFactory().create(300);
		while (true) {
			leftVel = leftCalc.step(leftDrive.getPosition()).convert(okapi::rpm);
			rightVel = rightCalc.step(rightDrive.getPosition()).convert(okapi::rpm);
			pros::delay(20);
		}
	});

	PIDController distPID(decelGains);
	PIDController headingPID(headingGains);

	double currDist = 0;
	int i = 0;
	uint32_t time = pros::millis(), st = pros::millis(), t = pros::millis();
	int dir = Util::sgn(dist);
	while (std::abs(dist) - currDist > 0.5) {
		t = pros::millis();
		double headingCtrl = headingPID.calculate(heading, inertial.get_rotation());
		// double avgTicks = std::abs(leftEncoder.get() + rightEncoder.get()) / 2.0;
		// currDist = avgTicks / 360.0 * 2.75 * M_PI;
		double avgTicks = std::abs(leftDrive.getPosition() + rightDrive.getPosition()) / 2.0;
		currDist = (avgTicks / 300.0) * 4 * M_PI * (4/6.0);
		double distCtrl = distPID.calculate(dist, currDist);

		double accelRPM = (t - st <= profile.accelTime * 1000 ? profile[i++].velocity * 60 / (M_PI * 4) * (4/6.0) : maxRPM);
		double decelRPM = maxRPM * distCtrl;
		double vel = std::min(maxRPM, std::min(accelRPM, decelRPM));
		leftDrive.moveVoltage(((dir * vel) + (turnRPM * headingCtrl)) / 600.0 * 12000);
		rightDrive.moveVoltage(((dir * vel) - (turnRPM * headingCtrl)) / 600.0 * 12000);
		// leftDrive.moveVelocity(vel);
		// rightDrive.moveVelocity(vel);
		// if (distCtrl < 1 && distCtrl > 0.7) vel = -50;
		// leftDrive.moveVoltage(vel / 600.0 * 12000);
		// rightDrive.moveVoltage(vel / 600.0 * 12000);
		std::cout << currDist << " " << leftVel << " " << rightVel << " " << vel << "\n";

		pros::Task::delay_until(&time, 20);
	}

	leftDrive.moveVelocity(0);
	rightDrive.moveVelocity(0);
}

void moveDistanceProfiledPID(double dist, MotionLimit motionLimit, double kV, double kA, double kP) {
	double mtrVel = 0;
	double encVel = 0;
	double encDist = 0;
	pros::Task vel = pros::Task([&] {
		okapi::VelMath mtrVelCalc = okapi::VelMathFactory().create(900);
		okapi::VelMath encVelCalc = okapi::VelMathFactory().create(360);
		while (true) {
			mtrVel = mtrVelCalc.step((leftDrive.getPosition() + rightDrive.getPosition()) / 2.0).convert(okapi::rpm);
			// encVel = encVelCalc.step((leftEncoder.get() + rightEncoder.get()) / 2.0).convert(okapi::rpm) * (2/6.0);
			// encDist = (leftEncoder.get() + rightEncoder.get()) / 2.0 / 360.0 * 2.75 * M_PI;
			if (isnan(mtrVel)) mtrVel = 0;
			if (isnan(encVel)) encVel = 0;
			if (isnan(encDist)) encDist = 0;
			// std::cout << mtrVel << " " << encVel * (2/6.0) << " " << inertial.get_rotation() << "\n";
			pros::delay(10);
		}
	});

	motionLimit = motionLimit / (4/6.0);
	MotionProfile profile = MotionProfiling::generateTrapezoidal(dist, motionLimit);

	std::cout << "Time CalculatedDistance ActualDistance MotorVelocity EncoderVelocity TargetVelocity Output\n";

	for (auto& data : profile.profile) {
		double tgtRPM = data.velocity * 60 / (M_PI * 4.0) * (4/6.0);
		double error = tgtRPM - mtrVel;
		double output = std::clamp(kV * data.velocity + kA * data.acceleration + (kP * error), 0.0, 200.0);

		leftDrive.moveVoltage(output / 200.0 * 12000);
		rightDrive.moveVoltage(output / 200.0 * 12000);

		std::cout << data.time << " " << data.distance << " " << encDist << " " << mtrVel << " " << encVel << " " << tgtRPM << " " << output << "\n";
		pros::delay(20);
	}

	leftDrive.moveVelocity(0);
	rightDrive.moveVelocity(0);
}

void moveDistanceHalfLife(double dist, double maxRPM, double decelZone) {
	double currDist = 0;
	double prevVel = maxRPM;
	while (dist - currDist > 0) {
		// double avgTicks = abs(leftEncoder.get() + rightEncoder.get()) / 2.0;
		// currDist = avgTicks / 360.0 * WHEEL_DIAMETER * M_PI;
		double vel = dist - currDist > decelZone ? maxRPM : maxRPM * std::pow(0.5, (currDist - (dist - decelZone)) / (decelZone / 5.0));
		prevVel = vel;
		leftDrive.moveVoltage(vel / 200 * 12000);
		rightDrive.moveVoltage(vel / 200 * 12000);
		// leftDrive.moveVelocity(vel);
		// rightDrive.moveVelocity(vel);
		std::cout << currDist << " " << vel << " " << leftDrive.getActualVelocity() << " " << rightDrive.getActualVelocity() << "\n";
		pros::delay(50);
	}
	std::cout << "------\n" << currDist << "\n";

	leftDrive.moveVoltage(0);
	rightDrive.moveVoltage(0);
}

void turnAbsoluteABS(double targetHeading, double maxRPM, PIDGains pidGains, double minPower, bool reduceAngle = false) {
	uint st = pros::millis();
	double leftMtrVel, rightMtrVel;
	double absCtrl;
	// pros::Task vel = pros::Task([&] {
	// 	okapi::VelMath leftMtrVelCalc = okapi::VelMathFactory().create(300);
	// 	okapi::VelMath rightMtrVelCalc = okapi::VelMathFactory().create(300);
	// 	okapi::VelMath leftEncVelCalc = okapi::VelMathFactory().create(360);
	// 	okapi::VelMath rightEncVelCalc = okapi::VelMathFactory().create(360);
	// 	while (true) {
	// 		// for (int i = 0; i < 10; i++) {
	// 		// 	leftMtrVel[i] = leftMtrVelCalc.step(leftDrive.getPosition()).convert(okapi::rpm);
	// 		// 	rightMtrVel[i] = rightMtrVelCalc.step(rightDrive.getPosition()).convert(okapi::rpm);
	// 		// 	pros::delay(5);
	// 		// }
	// 		// for (int i = 0; i < 10; i++) {
	// 		// 	std::cout << (i * 2 <= ((1 - absCtrl))) << " " << leftMtrVel[i] << " " << rightMtrVel[i] << " " <<  1 - absCtrl << "\n";
	// 		// }

	// 		leftMtrVel = std::make_shared<double>(leftMtrVelCalc.step(leftDrive.getPosition()).convert(okapi::rpm));
	// 		rightMtrVel = std::make_shared<double>(rightMtrVelCalc.step(rightDrive.getPosition()).convert(okapi::rpm));

	// 		double leftEncVel = leftEncVelCalc.step(leftEncoder.get()).convert(okapi::rpm);
	// 		double leftEncInps = leftEncVel / 60.0 * (M_PI * 2.75);
	// 		double leftEncRPM = leftEncInps * 60 / (M_PI * 4.0) / (4/6.0);

	// 		double rightEncVel = rightEncVelCalc.step(rightEncoder.get()).convert(okapi::rpm);
	// 		double rightEncInps = rightEncVel / 60.0 * (M_PI * 2.75);
	// 		double rightEncRPM = rightEncInps * 60 / (M_PI * 4.0) / (4/6.0);

	// 		// std::cout << pros::millis() - st << " " << leftMtrVel << " " << rightMtrVel << " " << leftEncRPM << " " << rightEncRPM << " " << inertial.get_rotation() << " " << 1 - absCtrl << "\n";
	// 	}
	// });

	PIDController absPID = PIDController(pidGains, 1, 0);

	okapi::VelMath leftMtrVelCalc = okapi::VelMathFactory().create(300);
	okapi::VelMath rightMtrVelCalc = okapi::VelMathFactory().create(300);

	double heading = inertial.get_rotation();
	double prevError = 0;
	bool absComplete = false;
	int numInSettleZone = 0;
	int c = 0;
	while (numInSettleZone < 5) {
		heading = inertial.get_rotation();
		double error = targetHeading - heading;
		if (reduceAngle) error = Util::ReduceAngle::deg180(error);
		absCtrl = absPID.calculate(std::abs(error));
		// std::cout << error << " " << absCtrl << "\n";

		leftMtrVel = leftMtrVelCalc.step(leftDrive.getPosition()).convert(okapi::rpm);
		rightMtrVel = rightMtrVelCalc.step(rightDrive.getPosition()).convert(okapi::rpm);

		std::cout << leftMtrVel << " " << rightMtrVel << " " << inertial.get_rotation() << " " << absCtrl << " ";

		if (std::abs(targetHeading - heading) < 1) numInSettleZone++;
		else numInSettleZone = 0;

		if (absComplete) {
			leftDrive.moveVelocity(minPower * Util::sgn(targetHeading - heading));
			rightDrive.moveVelocity(-minPower * Util::sgn(targetHeading - heading));
			if (Util::sgn(targetHeading - heading) != Util::sgn(prevError)) break;
			pros::delay(15);
		} else if (absCtrl < 1) {
			leftDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
			rightDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
			leftDrive.moveVelocity(0);
			rightDrive.moveVelocity(0);
			pros::delay(25 * (1 - absCtrl));
			leftDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			rightDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			leftDrive.moveVoltage(0);
			rightDrive.moveVoltage(-0);
			pros::delay(25 * absCtrl);
			c++;
			if (leftMtrVel == 0 && rightMtrVel == 0) {
				absComplete = true;
				prevError = error;
			}
		} else {
			double vel = maxRPM * Util::sgn(error);
			leftDrive.moveVoltage(vel / 600 * 12000);
			rightDrive.moveVoltage(-vel / 600 * 12000);
			// std::cout << vel << "\n";
			pros::delay(15);
		}
		std::cout << c << "\n";
	}

	leftDrive.moveVelocity(0);
	rightDrive.moveVelocity(0);

	// pros::lcd::print(0, "Turned %f degrees in %d ms", targetHeading - heading, pros::millis() - st);
	std::cout << inertial.get_rotation() << " done\n";
}