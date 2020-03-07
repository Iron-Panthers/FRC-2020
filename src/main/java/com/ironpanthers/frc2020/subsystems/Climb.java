/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ironpanthers.frc2020.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

	private TalonFX climbLeft;
	private TalonFX climbRight;
	/**
	 * Creates a new Climb.
	 */
	public Climb() {
		climbLeft = new TalonFX(Constants.Climb.kClimbLeft);
		climbRight = new TalonFX(Constants.Climb.kClimbRight);
		climbLeft.configFactoryDefault();
		climbRight.configFactoryDefault();

		climbLeft.setInverted(Constants.Climb.kClimbInverted);
		climbRight.follow(climbLeft);
		climbRight.setInverted(InvertType.OpposeMaster);

		climbLeft.configOpenloopRamp(Constants.Climb.kClimbRamp);
		climbLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.Climb.kClimbCurrentLimit, Constants.Climb.kClimbCurrentPeak, Constants.Climb.kClimbCurrentDelay));
		climbLeft.config_kP(Constants.Climb.kClimbPIDSlot, Constants.Climb.kClimbP);
	}

	public void setPosition(int nativeUnits) {
		climbLeft.set(TalonFXControlMode.Position, nativeUnits);
	}

	public void setPower(double power) {
		climbLeft.set(TalonFXControlMode.PercentOutput, power);
	}

	public void climbUp() {
		setPower(Constants.Climb.kClimbUpPower);
	}

	public void climbDown() {
		setPower(Constants.Climb.kClimbDownPower);
	}

	public void stop() {
		setPower(0);
	}

	public int getPosition() {
		return climbLeft.getSelectedSensorPosition();
	}

	public void setZero() {
		climbLeft.setSelectedSensorPosition(0);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
