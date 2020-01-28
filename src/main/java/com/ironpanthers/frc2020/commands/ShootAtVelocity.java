/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class ShootAtVelocity extends CommandBase {
	private Shooter shooter;
	private ConveyorBelt conveyor;
	private int velocity;
	private int ballsHeld;
	/**
	 * Creates a new ShootAtVelocity.
	 */
	public ShootAtVelocity(Shooter shooter, ConveyorBelt conveyor, int velocity) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(shooter);
		this.shooter = shooter;
		this.velocity = velocity;
		this.conveyor = conveyor;
		ballsHeld = conveyor.ballsHeld;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		shooter.setVelocity(velocity);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (Math.abs(shooter.getVelocity() - velocity) < Constants.Shooter.SHOOTER_VELOCITY_THRESHOLD) {
			// Yeet one ball
			CommandScheduler.getInstance().schedule(new PreparentConveyor(conveyor));
			ballsHeld--;
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return ballsHeld == 0;
	}
}