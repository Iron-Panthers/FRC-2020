/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.shooter;

import com.ironpanthers.frc2020.subsystems.Arm;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Shooter;
import com.ironpanthers.frc2020.util.LimelightWrapper;
import com.ironpanthers.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterInterpolation extends CommandBase {
    private final Shooter shooter;
    private int velocity;
    private final int threshold;
    private LimelightWrapper lWrapper;
    private ConveyorBelt conveyorBelt;
    private Arm arm;

    /**
     * Creates a new ShootAtVelocity.
     */
    public ShooterInterpolation(Arm arm, Shooter shooter, int threshold, ConveyorBelt conveyorBelt,
            LimelightWrapper lWrapper) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.shooter = shooter;
        this.threshold = threshold;
        this.lWrapper = lWrapper;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (conveyorBelt.ballsHeld <= 0) { // we have no balls or mis-indexed TODO reconsider
            lWrapper.turnOffLight();
            cancel();
        }

        shooter.velocity = shooter.interpolateY(arm.getHorizontalDistance(), shooter.velocityTable);
        shooter.setVelocity(shooter.velocity);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("Horizontal Distance", arm.getHorizontalDistance());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            shooter.stopShooter();
            arm.stop();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Util.epsilonEquals(shooter.getVelocity(), velocity, threshold)) {
            return true;
        } else {
            return false;
        }
    }
}
