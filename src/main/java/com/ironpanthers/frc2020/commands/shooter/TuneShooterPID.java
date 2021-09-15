// THIS CLASS KEPT ONLY FOR TESTING PURPOSES

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.shooter;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TuneShooterPID extends CommandBase {
	/**
	 * Creates a new TuneShooterPID.
	 */
	private Shooter shooter;
	double p, i, d, f, vel;

	public TuneShooterPID(Shooter shooter) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(shooter);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		SmartDashboard.putNumber("Shooter P", Constants.Shooter.kP);
		SmartDashboard.putNumber("Shooter I", 0.0);
		SmartDashboard.putNumber("Shooter D", 0.0);
		SmartDashboard.putNumber("Shooter F", Constants.Shooter.kF);
		SmartDashboard.putNumber("Target Velocity", 0.0);
		p = SmartDashboard.getNumber("Shooter P", Constants.Shooter.kP);
		i = SmartDashboard.getNumber("Shooter I", 0.0);
		d = SmartDashboard.getNumber("Shooter D", 0.0);
		f = SmartDashboard.getNumber("Shooter F", Constants.Shooter.kF);
		vel = SmartDashboard.getNumber("Target Velocity", 0.0);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double nP = SmartDashboard.getNumber("Shooter P", Constants.Shooter.kP);
		double nI = SmartDashboard.getNumber("Shooter I", 0.0);
		double nD = SmartDashboard.getNumber("Shooter D", 0.0);
		double nF = SmartDashboard.getNumber("Shooter F", Constants.Shooter.kF);
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
			shooter.configPIDF(p, i, d, f, Constants.Shooter.kPIDIdx);
		}
		if (vel != nVel) {
			vel = nVel;
		}
		shooter.setVelocity(vel);
		SmartDashboard.putNumber("Actual Velocity", shooter.getVelocity());
		SmartDashboard.putNumber("Motor Voltage", shooter.getVoltage());
		SmartDashboard.putNumber("Motor Current", shooter.getCurrent());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		shooter.stopShooter();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
