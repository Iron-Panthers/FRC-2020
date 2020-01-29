/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class SetShooterVelocity extends CommandBase {
	private Shooter shooter;
	private int velocity;
	/**
	 * Creates a new ShootAtVelocity.
	 */
	public SetShooterVelocity(Shooter shooter, int velocity) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(shooter);
		this.shooter = shooter;
		this.velocity = velocity;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		shooter.setVelocity(velocity);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		shooter.stopShooter();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return Math.abs(shooter.getVelocity() - velocity) < Constants.Shooter.SHOOTER_VELOCITY_THRESHOLD;
			// Yeet one ball
	}
}
