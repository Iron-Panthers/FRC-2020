package com.ironpanthers.frc2020.commands;

import com.ironpanthers.frc2020.commands.arm.ArmHold;
import com.ironpanthers.frc2020.commands.arm.ArmToTarget;
import com.ironpanthers.frc2020.commands.shooter.SetShooterVelocityEmergency;

import com.ironpanthers.frc2020.subsystems.Arm;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Drive;
import com.ironpanthers.frc2020.subsystems.Shooter;
import com.ironpanthers.frc2020.util.LimelightWrapper;
import com.ironpanthers.frc2020.util.SteeringAdjuster;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FullShooterSequence extends ParallelCommandGroup {
    public FullShooterSequence(SteeringAdjuster steerer, Drive drive, Arm arm, double target, Shooter shooter,
            int threshold, ConveyorBelt conveyor, LimelightWrapper lWrapper, int velocity) {
        addCommands(
            // arm motion: move to ref -> hold
            new SequentialCommandGroup(
                new ArmToTarget(arm, target),
                new InstantCommand(arm::engageBrake, arm)
            ),
            // shooter: set ref entire time
            new SetShooterVelocityEmergency(shooter, velocity, threshold, conveyor, lWrapper)
        );
    }
}