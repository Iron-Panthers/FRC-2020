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

public class ConveyorToDefault extends CommandBase {
	/**
	 * Creates a new ConveyorToDefault.
	 */
	private boolean bannerSensor;
	private double encoderStartTicks;
	private ConveyorBelt conveyor;

	public ConveyorToDefault(ConveyorBelt conveyor) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(conveyor);
		this.conveyor = conveyor;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (conveyor.conveyorFull()) cancel();
		bannerSensor = conveyor.getBannerSensor();
		encoderStartTicks = conveyor.getPosition();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// After the inital banner sensor input, if it saw a ball, run the conveyor belt
		// forwards to the default state which has balls fully contained within the conveyor.
		// If the banner is not seen this means no ball was taken in and thus run the conveyor
		// backwards to the state prior to intaking.
		if (!bannerSensor) {
			conveyor.setPosition(encoderStartTicks + Constants.Conveyor.TICKS_PREP_DISTANCE);
		} 

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		conveyor.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (!bannerSensor) {
			return conveyor.getPosition() >= encoderStartTicks - Constants.Conveyor.TICKS_PREP_DISTANCE - Constants.Conveyor.TICK_ERROR_TOLLERANCE;
		} 
		return true;
	}
}
