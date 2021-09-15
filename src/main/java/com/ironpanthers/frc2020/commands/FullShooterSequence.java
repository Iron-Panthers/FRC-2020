package com.ironpanthers.frc2020.commands;

import com.ironpanthers.frc2020.commands.arm.ArmHold;
import com.ironpanthers.frc2020.commands.arm.ArmToTargetLL;
import com.ironpanthers.frc2020.commands.shooter.SetShooterVelocityEmergency;

/*----------------------------------------------------------------------------*/

/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import com.ironpanthers.frc2020.subsystems.Arm;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Drive;
import com.ironpanthers.frc2020.subsystems.Shooter;
import com.ironpanthers.frc2020.util.LightMode;
import com.ironpanthers.frc2020.util.LimelightWrapper;
import com.ironpanthers.frc2020.util.SteeringAdjuster;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FullShooterSequence extends SequentialCommandGroup {
    /**
     * Creates a new FullShooterSequence.
     */

    public FullShooterSequence(SteeringAdjuster steerer, Drive drive, Arm arm, double target, Shooter shooter,
            int threshold, ConveyorBelt conveyor, LimelightWrapper lWrapper, int velocity) {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());super();
        super(new ArmToTargetLL(arm, target, lWrapper, LightMode.ON), new ParallelCommandGroup(new ArmHold(arm),
                new SetShooterVelocityEmergency(shooter, velocity, threshold, conveyor, lWrapper)));
    }
}
