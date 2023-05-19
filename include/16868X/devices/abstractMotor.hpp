#pragma once
#include "16868X/devices/abstractEncoder.hpp"
#include "16868X/util/filters/emaFilter.hpp"
#include "16868X/util/filters/medianFilter.hpp"
#include "16868X/util/filters/rangeExtremaFilter.hpp"
#include "16868X/util/filters/smaFilter.hpp"
#include "okapi/api.hpp"

namespace lib16868X {

class AbstractMotor : public AbstractEncoder {
	public:
		enum class Cartridge {
			RED = 100,
			GREEN = 200,
			BLUE = 600,
			LOWWATT = 200
		};

		virtual void moveVelocityPID(double targetVel, double targetAccel) = 0;
		virtual void moveVelocityPID(double targetVel, double targetAccel, double kV, double kA, double kP) = 0;

		virtual void moveVelocity(double velocity) = 0;
		virtual void moveVoltage(double voltage) = 0;
		virtual void moveAbsolute(double position, double velocity) = 0;
		virtual void moveRelative(double position, double velocity) = 0;

		virtual double getActualVelocity() = 0;

		virtual int32_t getRawPosition(uint32_t* timestamp) = 0;
		
		virtual double getVelocity() = 0;
		virtual double getAcceleration() = 0;

		virtual int getMaxRPM() = 0;
		virtual bool isLowWatt() = 0;
	
	protected:
		double targetVel, targetAccel, kV, kA, kP;

		double vel, accel;

		SMAFilter smaVel {3};
		SMAFilter smaAccel {15};
		MedianFilter medianVel {7};
		RangeExtremaFilter maxAccel {20};
		EMAFilter emaVel {};
		int prevMtrTicks = 0;
		uint32_t prevMtrClock = 0;
		double prevCalcVel = 0, prevFilteredVel = 0;
		
		inline static okapi::AbstractMotor::gearset cartridgeToGearset(Cartridge cartridge) {
			switch (cartridge) {
				case Cartridge::RED:
					return okapi::AbstractMotor::gearset::red;
				case Cartridge::GREEN: // and Cartridge::LOWWATT (same rpm, indistinguishable)
					return okapi::AbstractMotor::gearset::green;
				case Cartridge::BLUE:
					return okapi::AbstractMotor::gearset::blue;
			}
			return okapi::AbstractMotor::gearset::green;
		}
		inline static Cartridge gearsetToCartridge(okapi::AbstractMotor::gearset gearset) {
			switch (gearset) {
				case okapi::AbstractMotor::gearset::red:
					return Cartridge::RED;
				case okapi::AbstractMotor::gearset::green:
					return Cartridge::GREEN;
				case okapi::AbstractMotor::gearset::blue:
					return Cartridge::BLUE;
				case okapi::AbstractMotor::gearset::invalid:
					return Cartridge::GREEN;
			}
			return Cartridge::GREEN;
		}

		static double calcVelocity(std::shared_ptr<lib16868X::AbstractMotor> mtr);
		static void velocityPID(std::shared_ptr<lib16868X::AbstractMotor> mtr);

		static pros::task_t mtrManagerTask;
		static void mtrManager(void* param);
		static std::vector<std::shared_ptr<lib16868X::AbstractMotor>> mtrs;
		static void addMotor(std::shared_ptr<lib16868X::AbstractMotor> mtr);

		friend void mtrManager(void* param);
};
} // namespace lib16868X