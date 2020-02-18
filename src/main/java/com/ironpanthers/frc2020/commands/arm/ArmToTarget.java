/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.arm;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.Arm;
import com.ironpanthers.frc2020.util.LimelightWrapper;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmToTarget extends CommandBase {
	private Arm arm;
	private int target;
	private LimelightWrapper lWrapper;

	/**
	 * Creates a new ArmToTarget.
	 */
	public ArmToTarget(Arm arm, int target, LimelightWrapper lWrapper) {
		this.arm = arm;
		this.target = target;
		this.lWrapper = lWrapper;
		addRequirements(arm);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		arm.setPosition(target);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		lWrapper.turnOnLight();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return Math.abs(arm.getPosition() - target) < Constants.Arm.kPositionErrorTolerance;
	}
}
