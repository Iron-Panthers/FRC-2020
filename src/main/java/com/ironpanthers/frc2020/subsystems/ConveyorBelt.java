/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ironpanthers.frc2020.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorBelt extends SubsystemBase {
	public TalonFX conveyorMotor;
	public DigitalInput input;
	public boolean lastBallRan;

	// TODO only conditionally initialize with 3?
	public int ballsHeld = 0;
	
	private boolean usingPositionGains;

	/**
	 * Create a new ConveyorBelt subsystem. As usual, only one of these should ever
	 * be constructed.
	 */
	public ConveyorBelt() {
		input = new DigitalInput(Constants.Conveyor.kBannerSensorPort);
		conveyorMotor = new TalonFX(Constants.Conveyor.kConveyorMotorId);
		conveyorMotor.setInverted(false);

		conveyorMotor.config_kP(Constants.Conveyor.kPIDIdx, Constants.Conveyor.kConveyorPositionKp);
		usingPositionGains = true;

		conveyorMotor.configClosedloopRamp(Constants.Conveyor.kConveyorClosedLoopRamp);
		conveyorMotor.setSelectedSensorPosition(0);
	}

	private void config_kP(boolean positionControl) {
		if (positionControl)
			conveyorMotor.config_kP(Constants.Conveyor.kPIDIdx, Constants.Conveyor.kConveyorPositionKp);
		else
			conveyorMotor.config_kP(Constants.Conveyor.kPIDIdx, Constants.Conveyor.kConveyorVelocityKp);

	}

	public int getPosition() {
		return conveyorMotor.getSelectedSensorPosition();
	}

	public boolean getBannerSensor() {
		return input.get();
	}

	public void setVelocity(double velocitySTU) {
		if (usingPositionGains)
			config_kP(false);

		conveyorMotor.set(TalonFXControlMode.Velocity, velocitySTU);
	}

	public void setPosition(double ticks) {
		if (!usingPositionGains)
			config_kP(true);

		conveyorMotor.set(TalonFXControlMode.Position, ticks);
	}

	public void setPower(double power) {
		conveyorMotor.set(TalonFXControlMode.PercentOutput, power);
	}

	public void stop() {
		setPower(0);
	}

	public boolean conveyorFull() {
		return ballsHeld >= 5;
	}

	@Override
	public void periodic() {
		// SmartDashboard.putNumber("ballsHeld", ballsHeld);
		// SmartDashboard.putBoolean("lastBallRan", lastBallRan);

		// This method will be called once per scheduler run
	}
}
