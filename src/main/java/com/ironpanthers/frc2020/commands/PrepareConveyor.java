/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PrepareConveyor extends CommandBase {
	private ConveyorBelt conveyor;
	private double encoderStartTicks;

	/**
	 * Creates a new PrepareConveyor.
	 */
	public PrepareConveyor(ConveyorBelt conveyor) {
		this.conveyor = conveyor;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(conveyor);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (conveyor.conveyorFull())
			cancel();

		encoderStartTicks = conveyor.encoder.getPosition();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		conveyor.setPower(Constants.Conveyor.CONVEYOR_BELT_MOTOR_POWER);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		conveyor.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {

		return (conveyor.encoder.getPosition() - encoderStartTicks) > Constants.Conveyor.PREPARATION_DISTANCE;
	}
}
