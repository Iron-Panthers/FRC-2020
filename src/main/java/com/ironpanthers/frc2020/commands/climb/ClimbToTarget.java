/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.climb;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.Climb;
import com.ironpanthers.util.CircularBuffer;
import com.ironpanthers.util.Util;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbToTarget extends CommandBase {

	private Climb climb;
	private CircularBuffer buffer;
	private int target;

	/**
	 * Creates a new DeployClimb.
	 */
	public ClimbToTarget(Climb climb, int target) {
		this.climb = climb;
		buffer = new CircularBuffer(25);
		this.target = target;
		addRequirements(climb);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		climb.setZero();
		buffer.clear();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		climb.setPosition(target);
		buffer.addValue(climb.getPosition());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		climb.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return Util.epsilonEquals(buffer.getAverage(), target, Constants.Climb.kClimbTolarance);
	}
}
