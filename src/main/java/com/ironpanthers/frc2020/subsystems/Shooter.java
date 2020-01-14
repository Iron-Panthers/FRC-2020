/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.ironpanthers.frc2020.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
	/**
	 * Creates a new Shooter.
	 */
	private TalonFX shooter1;
	private TalonFX shooter2;
	private TalonFX shooter3;

	public Shooter() {
		shooter1 = new TalonFX(Constants.Shooter.SHOOTER_ONE_PORT);
		shooter2 = new TalonFX(Constants.Shooter.SHOOTER_TWO_PORT);
		shooter3 = new TalonFX(Constants.Shooter.SHOOTER_THREE_PORT);
		shooter2.follow(shooter1);
		shooter3.follow(shooter2);
		shooter1.setInverted(false);
		shooter2.setInverted(false);
		shooter3.setInverted(false);
	}

	public void setPercentOutput(double percentage) {
		shooter1.set(TalonFXControlMode.PercentOutput, percentage);
	}

	public void setVelocity(double nativeUnits) {
		shooter1.set(TalonFXControlMode.Velocity, nativeUnits);
	}

	public void configPIDF(double p, double i, double d, double f, int idx) {
		shooter1.config_kP(idx, p);
		shooter1.config_kI(idx, i);
		shooter1.config_kD(idx, d);
		shooter1.config_kF(idx, f);
	}

	public void stop() {
		setPercentOutput(0);
	}

	public double getVelocity() {
		return shooter1.getSelectedSensorVelocity();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
