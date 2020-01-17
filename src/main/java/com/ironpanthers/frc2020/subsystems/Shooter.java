/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ironpanthers.frc2020.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
	/**
	 * Creates a new Shooter.
	 */

	public final TalonFX shooter1;
	public final TalonFX shooter2;
	public final TalonFX shooter3;

	public Shooter() {
		shooter1 = new TalonFX(Ports.shooter1);
		shooter2 = new TalonFX(Ports.shooter2);
		shooter3 = new TalonFX(Ports.shooter3);
		SupplyCurrentLimitConfiguration currentConfig = new SupplyCurrentLimitConfiguration(true, Constants.SHOOTER_CURRENT_LIMIT, 0, 0);
		shooter1.configSupplyCurrentLimit(currentConfig);
		shooter2.follow(shooter1);
		shooter3.follow(shooter1);
	}

	public void shootWithSpeed(double speed) {
		shooter1.set(ControlMode.PercentOutput, speed);
	}

	public void stopShooter() {
		shooter1.set(ControlMode.PercentOutput, 0);
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
		shootWithSpeed(0);
	}

	public double getVelocity() {
		return shooter1.getSelectedSensorVelocity();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
