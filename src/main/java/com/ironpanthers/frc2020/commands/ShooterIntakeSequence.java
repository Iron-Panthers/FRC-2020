/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands;

import java.util.function.BooleanSupplier;

import com.ironpanthers.frc2020.commands.ShiftConveyor.Direction;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShooterIntakeSequence extends SequentialCommandGroup {
	public ShooterIntakeSequence(Shooter shooter, ConveyorBelt conveyor, BooleanSupplier button) {
		super(new ShooterIntake(shooter, conveyor, button), new ShiftConveyor(Direction.kIn, conveyor));
	}
}
