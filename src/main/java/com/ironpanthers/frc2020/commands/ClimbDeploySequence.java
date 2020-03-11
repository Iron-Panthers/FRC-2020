/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.commands.arm.ArmToTarget;
import com.ironpanthers.frc2020.commands.climb.ClimbToTarget;
import com.ironpanthers.frc2020.subsystems.Arm;
import com.ironpanthers.frc2020.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ClimbDeploySequence extends ParallelCommandGroup {
	/**
	 * Creates a new ClimbDeploySequence.
	 */
	public ClimbDeploySequence(Climb climb, Arm arm) {
		// Add your commands in the super() call, e.g.
		// super(new FooCommand(), new BarCommand());
		super(new ArmToTarget(arm, Constants.Arm.kClimbDegrees),
				new ClimbToTarget(climb, Constants.Climb.kClimbUnwindDistance));
	}
}
