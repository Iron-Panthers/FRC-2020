/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.arm;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.Arm;
import com.ironpanthers.frc2020.util.LightMode;
import com.ironpanthers.frc2020.util.LimelightWrapper;
import com.ironpanthers.util.CircularBuffer;
import com.ironpanthers.util.Util;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmToTargetLL extends CommandBase {
	private Arm arm;
	private double target;
	private LimelightWrapper lWrapper;
	private LightMode mode;
	private CircularBuffer buffer;

	/**
	 * Creates a new ArmToTarget.
	 */
	public ArmToTargetLL(Arm arm, double target, LimelightWrapper lWrapper,LightMode mode) {
		this.arm = arm;
		this.target = target;
		this.lWrapper = lWrapper;
		this.mode = mode;
		buffer = new CircularBuffer(25);
		addRequirements(arm);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		buffer.clear();
		lWrapper.setLightMode(mode);
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
