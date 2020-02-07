/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.vision;

import com.ironpanthers.frc2020.Robot;
import com.ironpanthers.frc2020.util.LimelightWrapper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class VisionTest extends CommandBase {
    private final LimelightWrapper limelightWrapper;

    /**
     * Create a new vision testing command.
     * 
     * @param limelightWrapper The instance of LimelightWrapper for this command to
     *                         use.
     */
    public VisionTest(LimelightWrapper limelightWrapper) {
        this.limelightWrapper = limelightWrapper;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Update values
        limelightWrapper.periodic();

        // Put the perceived horizontal distance
        SmartDashboard.putNumber("Horizontal distance: ", limelightWrapper.calculateHorizontalDistance());

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
