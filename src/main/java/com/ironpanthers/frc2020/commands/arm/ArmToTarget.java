/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.arm;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.Arm;
import com.ironpanthers.util.CircularBuffer;
import com.ironpanthers.util.Util;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmToTarget extends CommandBase {
	private Arm arm;
	private double target;
	private CircularBuffer buffer;

	/**
	 * Creates a new ArmToTarget.
	 * @param double target in degrees
	 */
	public ArmToTarget(Arm arm, double angle) {
		this.arm = arm;
		this.target = angle;
		buffer = new CircularBuffer(25);
		addRequirements(arm);
		
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (Util.epsilonEquals(arm.getAngle(), target, Constants.Arm.kPositionErrorTolerance)) {
			this.cancel();
		}
		
		buffer.clear();
		arm.releaseBrake();
		arm.calibrateCANCoder();
		// Only in endgame go above 45 inches
		if (target == Constants.Arm.kClimbDegrees) {
			arm.configureForwardSoftLimit(Constants.Arm.kTopSoftLimitEndgame);
		}
		else {
			arm.configureForwardSoftLimit(Constants.Arm.kTopSoftLimit);
		}
		arm.setPosition(target);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		buffer.addValue(arm.getAngle());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		arm.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return Util.epsilonEquals(buffer.getAverage(), target, Constants.Arm.kPositionErrorTolerance);
	}
}
