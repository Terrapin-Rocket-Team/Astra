// ODrive motor control surface implementation

#ifndef ODRIVE_MOTOR_H
#define ODRIVE_MOTOR_H

#include "ControlSurfaces/ControlSurface.h"
#include <Arduino.h>
#include <ODriveUART.h>

namespace astra
{
	enum class OdriveStalledState
	{
		STOPPED = 0,
		TOP = 1,
		BOTTOM = 2,
		MOVING = 3,
		ERROR = 4
	};

	/**
	 * Configuration for ODrive motor control
	 */
	struct OdriveMotorConfig : public ControlSurfaceConfig
	{
		HardwareSerial* serial = &Serial1;
		unsigned long baudrate = 115200;
		ODriveUART* odrive = nullptr;

		int topLimitSwitchPin = 35;  // Active LOW limit switch

		float minPosition = 0.0f;
		float maxPosition = 26.0f;
		float minAngle = 0.0f;
		float maxAngle = 80.0f;

		bool invertDirection = true;
		bool useClosedLoop = true;
		bool idleOnEStop = false;

		uint32_t initTimeoutMs = 10000;
		uint32_t stateRetryDelayMs = 1000;
		float zeroVelocity = 1.0f;

		int stallSampleCount = 10;
		float stallPositionEpsilon = 0.0001f;
	};

	/**
	 * OdriveMotor - ODrive UART motor control
	 *
	 * Normalized position: 0.0 = minPosition, 1.0 = maxPosition
	 * Raw position: position units defined by ODrive (typically turns)
	 */
	class OdriveMotor : public ControlSurface
	{
	public:
		OdriveMotor(const char* name = "OdriveMotor");
		virtual ~OdriveMotor() = default;

		// ControlSurface interface implementation
		bool setNormalizedPosition(float normalized) override;
		float getNormalizedPosition() const override;
		bool setRawPosition(float raw) override;
		float getRawPosition() const override;
		bool zero() override;
		bool isWithinLimits() const override;
		bool EStop() override;

		/**
		 * Convenience methods
		 */
		bool updateFeedback();
		float getPosition() const;
		float getVelocity() const;
		float getAngle() const;

		bool setPosition(float pos);
		bool setVelocity(float vel);

		float angleToPos(float angle) const;
		float posToAngle(float pos) const;

		bool motorStall();
		bool zeroMotor();

	protected:
		int init(const ControlSurfaceConfig* config) override;

		HardwareSerial* serial;
		ODriveUART* odrive;

		unsigned long baudrate;
		int topLimitSwitchPin;

		float minPosition;
		float maxPosition;
		float minAngle;
		float maxAngle;
		float directionSign;

		bool useClosedLoop;
		bool idleOnEStop;

		uint32_t initTimeoutMs;
		uint32_t stateRetryDelayMs;
		float zeroVelocity;

		int stallSampleCount;
		float stallPositionEpsilon;

		static constexpr int kMaxStallSamples = 32;

		ODriveFeedback feedback;
		float positionHistory[kMaxStallSamples];
		int positionHistoryCount;
		int positionHistoryIndex;

		float position;
		float angle;
		float velocity;
		float initPositionOffset;
		float targetPosition;
		float targetVelocity;
		float voltage;

		OdriveStalledState stalledState;
		int stalledStateTelemetry;

		void resetPositionHistory();
		void pushPositionHistory(float value);
	};

} // namespace astra

#endif // ODRIVE_MOTOR_H
