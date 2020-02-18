package com.ironpanthers.frc2020.commands;


import com.ironpanthers.frc2020.commands.ShiftConveyor.Direction;
import com.ironpanthers.frc2020.commands.arm.ArmToTarget;

/*----------------------------------------------------------------------------*/

/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import com.ironpanthers.frc2020.commands.shooter.SetShooterVelocity;
import com.ironpanthers.frc2020.commands.vision.TurnToTarget;
import com.ironpanthers.frc2020.subsystems.Arm;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Drive;
import com.ironpanthers.frc2020.subsystems.Shooter;
import com.ironpanthers.frc2020.util.LimelightWrapper;
import com.ironpanthers.frc2020.util.SteeringAdjuster;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FullShooterSequence extends SequentialCommandGroup {
    /**
     * Creates a new FullShooterSequence.
     */

    public FullShooterSequence(SteeringAdjuster steerer,
            Drive drive, Arm arm, int target, Shooter shooter, int velocity, int threshold, ConveyorBelt conveyor,
            LimelightWrapper lWrapper) {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());super();
        super(new ArmToTarget(arm, target, lWrapper), new TurnToTarget(drive, steerer, lWrapper),
                new SetShooterVelocity(shooter, velocity, threshold, conveyor, lWrapper),
                new ShiftConveyor(Direction.kOut, conveyor, shooter, velocity, threshold, lWrapper));
    }
}
