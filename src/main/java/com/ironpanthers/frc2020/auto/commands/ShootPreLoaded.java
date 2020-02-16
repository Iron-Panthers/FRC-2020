/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.auto.commands;

import com.ironpanthers.frc2020.commands.arm.ArmAndSpinShooter;
import com.ironpanthers.frc2020.commands.arm.ArmToTarget;
import com.ironpanthers.frc2020.commands.shooter.ShooterSequence;
import com.ironpanthers.frc2020.subsystems.Arm;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootPreLoaded extends ParallelCommandGroup {
	/**
	 * Creates a new ShootPreLoaded.
	 */
	public ShootPreLoaded(Arm arm, int target, Shooter shooter, int velocity, int threshold, ConveyorBelt conveyor) {
		// Add your commands in the super() call, e.g.
		// super(new FooCommand(), new BarCommand());
		super(new ArmToTarget(arm, target), new ShooterSequence(shooter, conveyor, velocity, threshold));
	}
}
