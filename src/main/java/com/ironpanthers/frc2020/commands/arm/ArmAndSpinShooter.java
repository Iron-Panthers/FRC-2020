/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.arm;

import com.ironpanthers.frc2020.commands.shooter.SetShooterVelocity;
import com.ironpanthers.frc2020.subsystems.Arm;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Shooter;
import com.ironpanthers.frc2020.util.LightMode;
import com.ironpanthers.frc2020.util.LimelightWrapper;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ArmAndSpinShooter extends ParallelCommandGroup {
	/**
	 * Creates a new ArmAndSpinShooter.
	 */
	public ArmAndSpinShooter(Arm arm, double target, Shooter shooter, int velocity, int threshold, ConveyorBelt conveyor, LimelightWrapper lWrapper) {
		// Add your commands in the super() call, e.g.
		// super(new FooCommand(), new BarCommand());super();
		super(new ArmToTargetLL(arm, target, lWrapper, LightMode.ON),
				new SetShooterVelocity(shooter, velocity, threshold, conveyor, lWrapper));
	}
}
