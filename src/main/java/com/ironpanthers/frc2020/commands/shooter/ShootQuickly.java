/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.shooter;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.commands.ShiftConveyor;
import com.ironpanthers.frc2020.commands.ShiftConveyor.Direction;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Shooter;
import com.ironpanthers.frc2020.util.LimelightWrapper;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootQuickly extends SequentialCommandGroup {
	/**
	 * Creates a new IntakeSequence.
	 */
	public ShootQuickly(Shooter shooter, ConveyorBelt conveyor, int velocity, int threshold, LimelightWrapper lWrapper) {
		// Add your commands in the super() call, e.g.
		// super(new FooCommand(), new BarCommand());
	super(new SetShooterVelocity(shooter, Constants.Shooter.kFarVelocity, threshold, conveyor, lWrapper), new ShiftConveyor(Direction.kOut, conveyor),new ShiftConveyor(Direction.kOut, conveyor),new ShiftConveyor(Direction.kOut, conveyor),new ShiftConveyor(Direction.kOut, conveyor),new ShiftConveyor(Direction.kOut, conveyor));
	
	}
}
