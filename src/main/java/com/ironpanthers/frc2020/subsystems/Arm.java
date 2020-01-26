/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ironpanthers.frc2020.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
	private TalonFX armLeft;
	private TalonFX armRight;
	/**
	 * Creates a new Arm.
	 */
	public Arm() {
		armLeft = new TalonFX(Constants.Arm.ARM_LEFT_PORT);
		armRight = new TalonFX(Constants.Arm.ARM_RIGHT_PORT);
		armLeft.setSensorPhase(false);
		armLeft.setInverted(Constants.Arm.IS_LEFT_ARM_INVERTED);
		armRight.setInverted(Constants.Arm.IS_RIGHT_ARM_INVERTED);
		armLeft.setNeutralMode(NeutralMode.Brake);
		armRight.setNeutralMode(NeutralMode.Brake);
		armRight.follow(armLeft);
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
	 * Set velocity of the arm, will be used in tuning PID for velocity to get values for MotionMagic
	 */
	public void setVelocity(double nativeUnits) {
		armLeft.set(TalonFXControlMode.Velocity, nativeUnits);
	}

	public void setPosition(int target) {
		armLeft.set(TalonFXControlMode.Position, target);
	}

	/**
	 * Motion Magic may not be needed, position is probably good enough
	 */
	public void setMotionMagicPosition(int target) {
		armLeft.set(TalonFXControlMode.MotionMagic, target);
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

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
