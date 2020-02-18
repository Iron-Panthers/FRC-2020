/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.shooter;

import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Shooter;
import com.ironpanthers.frc2020.util.LimelightWrapper;
import com.ironpanthers.util.Util;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetShooterVelocity extends CommandBase {
	private final Shooter shooter;
	private int velocity;
	private final int threshold;
	private ConveyorBelt conveyorBelt;
	private LimelightWrapper lWrapper;

	/**
	 * Creates a new ShootAtVelocity.
	 */
	public SetShooterVelocity(Shooter shooter, int velocity, int threshold, ConveyorBelt conveyorBelt, LimelightWrapper lWrapper) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.shooter = shooter;
		this.velocity = velocity;
		this.threshold = threshold;
		this.conveyorBelt = conveyorBelt;
		this.lWrapper = lWrapper;
		addRequirements(shooter);
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
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// velocity = (int) SmartDashboard.getNumber("Shooter Test Velocity", Constants.Shooter.kTestVelocity);
		shooter.setVelocity(velocity);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			shooter.stopShooter();
		}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		 if (Util.epsilonEquals(shooter.getVelocity(), velocity, threshold)) {
			conveyorBelt.ballsHeld--;
			return true;
		} else {
			return false;
		}
	}
}
