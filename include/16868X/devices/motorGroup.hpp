#pragma once
#include "16868X/devices/abstractMotor.hpp"
#include "16868X/devices/motor.hpp"
#include <initializer_list>
#include <memory>
#include <vector>

namespace lib16868X {
class MotorGroup : public AbstractMotor {
	public:
		MotorGroup(const MotorGroup& motorGroup);
		MotorGroup(std::initializer_list<Motor> motors);
		MotorGroup(std::vector<Motor> motors);

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
			for (auto& motor : motors) {
				motor->okapi::Motor::moveVelocity(velocity);
			}
		}
		inline void moveVoltage(double voltage) override {
			for (auto& motor : motors) {
				motor->okapi::Motor::moveVoltage(voltage);
			}
		}
		inline void moveAbsolute(double position, double velocity) override {
			for (auto& motor : motors) {
				motor->okapi::Motor::moveAbsolute(position, velocity);
			}
		}
		inline void moveRelative(double position, double velocity) override {
			for (auto& motor : motors) {
				motor->okapi::Motor::moveRelative(position, velocity);
			}
		}

		inline double getActualVelocity() override {
			return motors[0]->getVelocity();
		}

		inline int32_t getRawPosition(uint32_t* timestamp) override {
			return motors[0]->getRawPosition(timestamp);
		}

		inline double get() override {
			return motors[0]->get();
		}
		inline void resetZero() override {
			for (auto& motor : motors) {
				motor->resetZero();
			}
		}
		inline double getVelocity() override {
			return motors[0]->getVelocity();
		}
		inline double getAcceleration() override {
			return motors[0]->getAcceleration();
		}

		inline Cartridge getCartridge() const {
			return cartridge;
		}
		inline int getMaxRPM() override {
			return static_cast<int>(cartridge);
		}
		inline bool isLowWatt() override {
			if (copied) {
				std::cerr << "A copied motor object cannot be checked whether it is 5.5W or 11W.\n";
				return false;
			}
			return cartridge == Cartridge::LOWWATT;
		}

		inline std::shared_ptr<Motor> operator[](int index) {
			return motors[index];
		}

		/** Default motor functions **/
		inline double getTemperature() {
			return motors[0]->okapi::Motor::getTemperature();
		}
		inline bool isOverHeating() {
			for (auto& motor : motors) {
				if (motor->okapi::Motor::isOverTemp()) return true;
			}
			return false;
		}
		
		inline void setBrakeMode(okapi::AbstractMotor::brakeMode mode) {
			for (auto& motor : motors) {
				motor->okapi::Motor::setBrakeMode(mode);
			}
		}
		inline okapi::AbstractMotor::brakeMode getBrakeMode() {
			return motors[0]->okapi::Motor::getBrakeMode();
		}

	private:
		std::vector<std::shared_ptr<Motor>> motors;

		lib16868X::AbstractMotor::Cartridge cartridge;
		bool copied { false };
};
} // namespace lib16868X