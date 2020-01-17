/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TuneShooterPID extends CommandBase {
	/**
	 * Creates a new TuneShooterPID.
	 */
	double p, i, d, f, vel;
	public TuneShooterPID() {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(Robot.shooter);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		SmartDashboard.putNumber("Shooter P", Constants.Shooter.SHOOTER_P);
		SmartDashboard.putNumber("Shooter I", 0.0);
		SmartDashboard.putNumber("Shooter D", 0.0);
		SmartDashboard.putNumber("Shooter F", Constants.Shooter.SHOOTER_F);
		SmartDashboard.putNumber("Target Velocity", 0.0);
		p = SmartDashboard.getNumber("Shooter P", Constants.Shooter.SHOOTER_P);
		i = SmartDashboard.getNumber("Shooter I", 0.0);
		d = SmartDashboard.getNumber("Shooter D", 0.0);
		f = SmartDashboard.getNumber("Shooter F", Constants.Shooter.SHOOTER_F);
		vel = SmartDashboard.getNumber("Target Velocity", 0.0);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double nP = SmartDashboard.getNumber("Shooter P", Constants.Shooter.SHOOTER_P);
		double nI = SmartDashboard.getNumber("Shooter I", 0.0);
		double nD = SmartDashboard.getNumber("Shooter D", 0.0);
		double nF = SmartDashboard.getNumber("Shooter F", Constants.Shooter.SHOOTER_F);
		double nVel = SmartDashboard.getNumber("Target Velocity", 0.0);
		boolean isChanged = false;
		if (p != nP) {
			p = nP;
			isChanged = true;
		}
		if (i != nI) {
			i = nI;
			isChanged = true;
		}
		if (d != nD) {
			d = nD;
			isChanged = true;
		}
		if (f != nF) {
			f = nF;
			isChanged = true;
		}
		if (isChanged) {
			Robot.shooter.configPIDF(p, i, d, f, Constants.Shooter.SHOOTER_VELOCITY_IDX);
		}
		if (vel != nVel) {
			vel = nVel;
		}
		Robot.shooter.setVelocity(vel);
		SmartDashboard.putNumber("Actual Velocity", Robot.shooter.getVelocity());
		SmartDashboard.putNumber("Motor Voltage", Robot.shooter.getVoltage());
		SmartDashboard.putNumber("Motor Current", Robot.shooter.getCurrent());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Robot.shooter.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
