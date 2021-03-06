/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.vision;

import java.util.function.BooleanSupplier;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.Drive;
import com.ironpanthers.frc2020.util.LimelightWrapper;
import com.ironpanthers.frc2020.util.SteeringAdjuster;
import com.ironpanthers.util.CircularBuffer;
import com.ironpanthers.util.Util;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToTarget extends CommandBase {
	private final Drive drive;
	SteeringAdjuster steerer;

	BooleanSupplier seeTarget;

	LimelightWrapper lWrapper;

	private CircularBuffer buffer;

	public TurnToTarget(Drive drive, SteeringAdjuster steerer,
			LimelightWrapper limelightWrapper) {
		this.drive = drive;
		this.steerer = steerer;
		lWrapper = limelightWrapper;
		buffer = new CircularBuffer(25);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		buffer.clear();
		drive.shiftLow();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		//calls with 0.0 as param because we are aiming at the center of the target
		drive.setOutputPercent(-steerer.updateSteeringValues(0.0), steerer.updateSteeringValues(0.0));
		buffer.addValue(lWrapper.getTableX());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drive.shiftHigh();
		drive.setOutputPercent(0.0, 0.0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return (Util.epsilonEquals(buffer.getAveragePartiallyFilled(), 0, Constants.Vision.kAutoAlignTolerance));
	}
}
