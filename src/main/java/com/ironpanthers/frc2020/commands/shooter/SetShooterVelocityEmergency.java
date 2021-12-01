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

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetShooterVelocityEmergency extends CommandBase {
	private final Shooter shooter;
	private int velocity;
	private final int threshold;

	/**
	 * Creates a new ShootAtVelocity.
	 */
	public SetShooterVelocityEmergency(Shooter shooter, int velocity, int threshold, ConveyorBelt conveyorBelt,
			LimelightWrapper lWrapper) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.shooter = shooter;
		this.velocity = velocity;
		this.threshold = threshold;
		addRequirements(shooter);
		// SmartDashboard.putNumber("Shooter Test Velocity",
		// Constants.Shooter.kTestVelocity);
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
		return false;
	}
}
