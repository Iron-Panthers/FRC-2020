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
	public int ballsHeld;

	public ConveyorBelt() {
		input = new DigitalInput(Constants.Conveyor.BANNER_SENSOR_PORT);
		conveyorMotor = new TalonFX(Constants.Conveyor.CONVEYOR_BELT_MOTOR_PORT);
		conveyorMotor.config_kP(0, .13);
		conveyorMotor.configClosedloopRamp(.6);
		ballsHeld = 0;
		conveyorMotor.setSelectedSensorPosition(0);
	}

	public int getPosition() {
		return conveyorMotor.getSelectedSensorPosition();
	}

	public boolean getBannerSensor() {
		return input.get();
	}

	public void setPosition(double ticks) {
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

	// public static void moveOneBall() {
	// while (encoder.getDistance() < Constants.POWER_CELL_DIAMETER) {
	// motor.set(ControlMode.PercentOutput, Constants.CONVEYOR_BELT_MOTOR_POWER);
	// }
	// motor.set(ControlMode.PercentOutput, 0);
	// encoder.reset();
	// }

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
