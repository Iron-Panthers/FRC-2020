/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.Arm;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Shooter;
import com.ironpanthers.frc2020.util.LimelightWrapper;
import com.ironpanthers.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoArmShooter extends CommandBase {
	private final Shooter shooter;
	private int velocity;
	private final int threshold;
	private int height;
	private ConveyorBelt conveyorBelt;
	private LimelightWrapper lWrapper;
	private Arm arm;

	/**
	 * Creates a new ShootAtVelocity.
	 */
	public AutoArmShooter(Shooter shooter, int threshold, ConveyorBelt conveyorBelt, LimelightWrapper lWrapper, Arm arm) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.shooter = shooter;
		this.threshold = threshold;
		this.conveyorBelt = conveyorBelt;
		this.lWrapper = lWrapper;
		this.arm = arm;
		addRequirements(shooter, arm);
		// SmartDashboard.putNumber("Shooter Test Velocity", Constants.Shooter.kTestVelocity);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (conveyorBelt.ballsHeld <= 0) { // we have no balls or mis-indexed TODO reconsider
			lWrapper.turnOffLight();
			cancel();
		} else { // we have balls
			lWrapper.turnOnLight();
			arm.setPosition(Constants.Arm.kFrameConstrainedHeightNativeUnits);
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		SmartDashboard.putNumber("Horizontal Distance", arm.getHorizontalDistance());
		if (lWrapper.getTableY() != 0) {
			velocity = shooter.interpolateY(arm.getHorizontalDistance(), shooter.velocityTable);
			height = shooter.interpolateY(arm.getHorizontalDistance(), shooter.armPosTable);
			// velocity = (int) SmartDashboard.getNumber("Shooter Test Velocity", Constants.Shooter.kTestVelocity);
			shooter.setVelocity(velocity);
			arm.setPosition(height);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			shooter.stopShooter();
			arm.stop();
		}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		 if (Util.epsilonEquals(shooter.getVelocity(), velocity, threshold) && Util.epsilonEquals(arm.getPosition(), height, Constants.Arm.kPositionErrorTolerance)) {
			conveyorBelt.ballsHeld--; // This might be a problem when spamming a button
			return true;
		} else {
			return false;
		}
	}
}
