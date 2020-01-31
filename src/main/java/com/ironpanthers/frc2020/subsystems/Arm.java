/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ironpanthers.frc2020.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
	public static TalonFX armLeft;
	public static TalonFX armRight;
	private DigitalInput forwardLimitSwitch;
	private DigitalInput reverseLimitSwitch;
	private int target;
	private double horizontalHoldOuput; // TODO figure out this value by testing

	/**
	 * Creates a new Arm. For limits, forward refers to the front, in which the arm
	 * is all the way down and ready for intaking. Reverse refers to the back, in
	 * which the arm is all the way up at the maximum angle for shooting
	 */
	public Arm() {
		armLeft = new TalonFX(Constants.Arm.ARM_LEFT_PORT);
		armRight = new TalonFX(Constants.Arm.ARM_RIGHT_PORT);
		armLeft.setSensorPhase(false); // Up is positive
		armLeft.setInverted(Constants.Arm.IS_ARM_INVERTED);
		armRight.setInverted(InvertType.OpposeMaster);
		armLeft.setNeutralMode(NeutralMode.Brake);
		armRight.setNeutralMode(NeutralMode.Brake);
		armRight.follow(armLeft);

		// Current Limits and Power Limits
		armLeft.configClosedLoopPeakOutput(Constants.Arm.ARM_POSITION_PID_SLOT, Constants.Arm.MAX_ARM_PID_OUTPUT);
		SupplyCurrentLimitConfiguration currentConfig = new SupplyCurrentLimitConfiguration(true,
				Constants.Arm.ARM_CURRENT_LIMIT, Constants.Arm.ARM_CURRENT_LIMIT, 1);
		armLeft.configSupplyCurrentLimit(currentConfig);

		// Limit switches
		// Forward needs to be the highest positive value, so the high position
		// Reverse needs to be the lowest value, so the ground position
		forwardLimitSwitch = new DigitalInput(Constants.Arm.HIGH_LIMIT_SWITCH_PORT);
		reverseLimitSwitch = new DigitalInput(Constants.Arm.GROUND_LIMIT_SWTICH_PORT);
		armLeft.configForwardSoftLimitEnable(true);
		armLeft.configReverseSoftLimitEnable(true);
		armLeft.configForwardSoftLimitThreshold(Constants.Arm.TOP_SOFT_LIMIT);
		armLeft.configReverseSoftLimitThreshold(Constants.Arm.BOTTOM_SOFT_LIMIT);
	}

	public void setPower(double power) {
		armLeft.set(TalonFXControlMode.PercentOutput, power);
	}

	public void stop() {
		setPower(0);
	}

	public void configPIDF(double p, double i, double d, double f, int idx) {
		armLeft.config_kP(idx, p);
		armLeft.config_kI(idx, i);
		armLeft.config_kD(idx, d);
		armLeft.config_kF(idx, f);
	}

	/**
	 * Set velocity of the arm, will be used in tuning PID for velocity to get
	 * values for MotionMagic
	 */
	public void setVelocity(double nativeUnits) {
		armLeft.set(TalonFXControlMode.Velocity, nativeUnits);
	}

	public void setPosition(int target) {
		armLeft.set(TalonFXControlMode.Position, target);
		this.target = target;
	}

	/**
	 * Motion Magic may not be needed, position is probably good enough
	 */
	public void setMotionMagicPosition(int target) {
		armLeft.set(TalonFXControlMode.MotionMagic, target);
	}

	public void setFeedForward() {
		double scaledAngle = Math.cos(Math.toRadians(getCurrentAngle()));
		armLeft.set(ControlMode.MotionMagic, target, DemandType.ArbitraryFeedForward,
				Constants.Arm.MAX_FF * scaledAngle);
		SmartDashboard.putNumber("Arbitrary Feedforward", horizontalHoldOuput * scaledAngle);
	}

	// return angle in degrees
	public double getCurrentAngle() {
		double currentAngle = (armLeft.getSelectedSensorPosition() * Constants.Arm.TICKS_TO_DEGREES)
				+ Constants.Arm.ARM_ANGLE_OFFSET;
		SmartDashboard.putNumber("Current angle", currentAngle);
		// returns angle in degrees
		return currentAngle;
	}

	public int getPosition() {
		return armLeft.getSelectedSensorPosition();
	}

	public int getVelocity() {
		return armLeft.getSelectedSensorVelocity();
	}

	public double getOutputVoltage() {
		return armLeft.getMotorOutputVoltage();
	}

	public double getOutputCurrent() {
		return armLeft.getStatorCurrent();
	}

	public void setZero() {
		armLeft.setSelectedSensorPosition(0);
	}

	public boolean getGroundLimitPressed() {
		return !reverseLimitSwitch.get();
	}

	public boolean getHighLimitPressed() {
		return !forwardLimitSwitch.get();
	}

	@Override
	public void periodic() {
		if (getGroundLimitPressed()) {
			setZero();
		} else if (getHighLimitPressed()) {
			armLeft.setSelectedSensorPosition(Constants.Arm.TOP_ARM_POSITION);
		}
		SmartDashboard.putBoolean("High Limit", getHighLimitPressed());
		SmartDashboard.putBoolean("Ground Limit", getGroundLimitPressed());
		SmartDashboard.putNumber("Arm Position", getPosition());
		// // If within the slow threshold, limit output to scaled regular output
		// if (getPosition() < Constants.Arm.BOTTOM_SLOW_LIMIT || getPosition() > Constants.Arm.TOP_SLOW_LIMIT) {
		// 	armLeft.configClosedLoopPeakOutput(Constants.Arm.ARM_POSITION_PID_SLOT, Constants.Arm.SLOW_ARM_PID_OUTPUT);
		// }
		// // If out of the slow threshold, reset output to correct value
		// else {
		// 	armLeft.configClosedLoopPeakOutput(Constants.Arm.ARM_POSITION_PID_SLOT, Constants.Arm.MAX_ARM_PID_OUTPUT);
		// }
	}
}