// ODrive motor control surface implementation

#ifndef ODRIVE_MOTOR_H
#define ODRIVE_MOTOR_H

#include "ControlSurfaces/ControlSurface.h"
#include "Utils/CircBuffer.h"
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

  enum class MotorControlMode
  {
    POSITION,
    VELOCITY,
    TORQUE
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

    bool usingEncoder = false;

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
		float getPosition() const;
		float getVelocity() const;
    float getTorque() const;
    float getAngle() const;

    void setControl(MotorControlMode mode, float val);
    
    float angleToPos(float angle) const;
		float posToAngle(float pos) const;

		bool isStalled();
		bool zeroMotor();

    void updateSensors() override;

  private:
		int init(const ControlSurfaceConfig* config) override;
    
    bool setPosition(float pos);
    bool setVelocity(float vel);
    bool setTorque(float torque);

		HardwareSerial* serial;
		ODriveUART* odrive;

		unsigned long baudrate;
		int topLimitSwitchPin;

		float minPosition;
		float maxPosition;
		float minAngle;
		float maxAngle;
    // negative if direction should be inverted, 
    //    positive if direction should be normal
		float invert;

		bool useClosedLoop;
		bool idleOnEStop;

		uint32_t initTimeoutMs;
		uint32_t stateRetryDelayMs;
		float zeroVelocity;

		int stallSampleCount;
		float stallPositionEpsilon;


		ODriveFeedback feedback;
		CircBuffer<float> positionHistory;

		float position;
		float angle;
		float velocity;
    float initPositionOffset;
    float targetPosition;
		float targetVelocity;
    float targetTorque;
    float voltage;

    float encoderPosition;

    OdriveStalledState stalledState;
		int stalledStateTelemetry;

	};

} // namespace astra

#endif // ODRIVE_MOTOR_H
