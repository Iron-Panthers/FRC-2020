/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.drive;

import com.ironpanthers.frc2020.Robot;
import com.ironpanthers.frc2020.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GearShift extends CommandBase {

	private Drive drive;
	/**
	 * Creates a new GearShift.
	 */
	public GearShift(Drive drive) {
		this.drive = drive;
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		drive.shiftLow();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drive.shiftHigh();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
