/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ironpanthers.frc2020.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
	/**
	 * Creates a new Shooter.
	 */

	public final TalonFX shooter1;
	public final TalonFX shooter2;
	public final TalonFX shooter3;
	public final TalonFX intakeMotor;

	public Shooter() {
		shooter1 = new TalonFX(Constants.Shooter.SHOOTER_ONE_PORT);
		shooter2 = new TalonFX(Constants.Shooter.SHOOTER_TWO_PORT);
		shooter3 = new TalonFX(Constants.Shooter.SHOOTER_THREE_PORT);
		intakeMotor = new TalonFX(Constants.Conveyor.INTAKE_MOTOR_PORT);

		// Config
		intakeMotor.setNeutralMode(NeutralMode.Coast);
		shooter1.setNeutralMode(NeutralMode.Coast);
		shooter2.setNeutralMode(NeutralMode.Coast);
		shooter3.setNeutralMode(NeutralMode.Coast);
		SupplyCurrentLimitConfiguration currentConfig = new SupplyCurrentLimitConfiguration(true,
				Constants.Shooter.SHOOTER_CURRENT_LIMIT, 0, 0);
		shooter1.configSupplyCurrentLimit(currentConfig);
		shooter1.configClosedloopRamp(Constants.Shooter.SHOOTER_RAMP_RATE); // Ramp rate for Velocity PID
		shooter1.configOpenloopRamp(Constants.Shooter.SHOOTER_RAMP_RATE); // Ramp rate for open loop control

		// Follow
		shooter2.follow(shooter1);
		shooter3.follow(shooter1);
	}

	public void setIntakeMotors(double intakeMotorSpeed, double ShooterMotorSpeed) {
		intakeMotor.set(ControlMode.PercentOutput, intakeMotorSpeed);
		shooter1.set(ControlMode.PercentOutput, ShooterMotorSpeed);
	}

	public void stopIntake() {
		setIntakeMotors(0, 0);
	}

	public void shootWithPower(double speed) {
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

	public double getVelocity() {
		return shooter1.getSelectedSensorVelocity();
	}

	public double getVoltage() {
		return shooter1.getMotorOutputVoltage();
	}

	public double getCurrent() {
		return shooter1.getStatorCurrent();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
