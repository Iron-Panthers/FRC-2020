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
	public TuneShooterPID() {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(Robot.shooter);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		SmartDashboard.putNumber("Shooter P", 0);
		SmartDashboard.putNumber("Shooter I", 0);
		SmartDashboard.putNumber("Shooter D", 0);
		SmartDashboard.putNumber("Shooter F", 0);
		SmartDashboard.putNumber("Target Velocity", 0);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		Robot.shooter.configPIDF(SmartDashboard.getNumber("Shooter P", 0), SmartDashboard.getNumber("Shooter I", 0), SmartDashboard.getNumber("Shooter D", 0), SmartDashboard.getNumber("Shooter F", 0), Constants.SHOOTER_VELOCITY_IDX);
		Robot.shooter.setVelocity(SmartDashboard.getNumber("Target Velocity", 0));
		SmartDashboard.putNumber("Actual Velocity", Robot.shooter.getVelocity());
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
