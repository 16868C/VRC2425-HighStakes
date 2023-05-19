#pragma once
#include "16868X/devices/abstractMotor.hpp"
#include "okapi/api.hpp"

namespace lib16868X {
class Motor : public okapi::Motor, public lib16868X::AbstractMotor {
	public:
		Motor(const Motor& mtr);
		Motor(int port, Cartridge gearset = Cartridge::GREEN);

		inline void moveVelocityPID(double targetVel, double targetAccel) override {
			this->targetVel = targetVel;
			this->targetAccel = targetAccel;
		}
		inline void moveVelocityPID(double targetVel, double targetAccel, double kV, double kA, double kP) override {
			this->targetVel = targetVel;
			this->targetAccel = targetAccel;
			this->kV = kV;
			this->kA = kA;
			this->kP = kP;
		}

		inline void moveVelocity(double velocity) override {
			okapi::Motor::moveVelocity(velocity);
		}
		inline void moveVoltage(double voltage) override {
			okapi::Motor::moveVoltage(voltage);
		}
		inline void moveAbsolute(double position, double velocity) override {
			okapi::Motor::moveAbsolute(position, velocity);
		}
		inline void moveRelative(double position, double velocity) override {
			okapi::Motor::moveRelative(position, velocity);
		}

		inline double getActualVelocity() override {
			return vel;
		}

		inline int32_t getRawPosition(uint32_t* timestamp) override {
			return okapi::Motor::getRawPosition(timestamp);
		}

		inline double get() override {
			return okapi::Motor::getEncoder()->get();
		}
		inline void resetZero() override {
			okapi::Motor::getEncoder()->reset();
		}
		inline double getVelocity() override {
			return vel;
		}
		inline double getAcceleration() override {
			return accel;
		}
		
		inline Cartridge getCartridge() const {
			return cartridge;
		}
		inline int getMaxRPM() {
			return static_cast<int>(cartridge);
		}
		inline bool isLowWatt() {
			if (copied) {
				std::cerr << "A copied motor object cannot be checked whether it is 5.5W or 11W.\n";
				return false;
			}
			return cartridge == Cartridge::LOWWATT;
		}
	
	private:
		lib16868X::AbstractMotor::Cartridge cartridge;
		bool copied { false };
};
} // namespace lib16868X