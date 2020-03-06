/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.intake;

import com.ironpanthers.frc2020.subsystems.ConveyorBelt;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetBallsHeld extends CommandBase {

	private final ConveyorBelt conveyorBelt;
	/**
	 * Creates a new ResetBallsHeld.
	 */
	public ResetBallsHeld(ConveyorBelt conveyorBelt) {
		this.conveyorBelt = conveyorBelt;
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		conveyorBelt.ballsHeld = 0;
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
