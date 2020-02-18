/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.intake;

import java.util.function.BooleanSupplier;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Intake extends CommandBase {
	/**
	 * Creates a new Intake.
	 */
	Shooter shooter;
	ConveyorBelt conveyor;
	BooleanSupplier button;

	public Intake(Shooter shooter, ConveyorBelt conveyor, BooleanSupplier button) {
		this.button = button;
		this.conveyor = conveyor;
		this.shooter = shooter;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(shooter, conveyor);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		conveyor.ballsHeld++;	
		if (conveyor.ballsHeld > 5) cancel();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		shooter.setIntakeMotors(Constants.Conveyor.kIntakeRollerSpeed, Constants.Conveyor.kIntakeFlywheelSpeed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		shooter.setIntakeMotors(0, 0);	
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (conveyor.getBannerSensor()) {
			return true;
		} else {
			return false;
		}
	}
}
