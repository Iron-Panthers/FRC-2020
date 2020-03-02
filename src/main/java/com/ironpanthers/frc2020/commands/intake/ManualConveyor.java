/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.intake;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualConveyor extends CommandBase {

	private final ConveyorBelt conveyor;
	
	/**
	 * Creates a new ManualConveyor.
	 */
	public ManualConveyor(ConveyorBelt conveyor) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.conveyor = conveyor;
		// addRequirements(conveyor); // DO NOT REQUIRE SUBSYSTEM
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		conveyor.setPower(Constants.Conveyor.kManualConveyorSpeed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		conveyor.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}