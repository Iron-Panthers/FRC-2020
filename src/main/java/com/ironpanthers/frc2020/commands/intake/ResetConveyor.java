/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.intake;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.util.Util;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetConveyor extends CommandBase {
	private boolean bannerSensor;
	private int targetEncoderPosition;
	private ConveyorBelt conveyor;

	public ResetConveyor(ConveyorBelt conveyor) {
		this.conveyor = conveyor;
		addRequirements(conveyor);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (conveyor.conveyorFull())
			cancel();
		bannerSensor = conveyor.getBannerSensor();
		targetEncoderPosition = conveyor.getPosition() + Constants.Conveyor.TICKS_PREP_DISTANCE;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// After the inital banner sensor input, if it saw a ball, run the conveyor belt
		// forwards to the default state which has balls fully contained within the
		// conveyor.
		// If the banner is not seen this means no ball was taken in and thus run the
		// conveyor
		// backwards to the state prior to intaking.
		if (!bannerSensor) {
			conveyor.setPosition(targetEncoderPosition);
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
			return Util.epsilonEquals(conveyor.getPosition(), targetEncoderPosition,
					Constants.Conveyor.TICK_ERROR_TOLERANCE);
		}
		return true;
	}
}
