/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.vision;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.Drive;
import com.ironpanthers.frc2020.util.LimelightWrapper;
import com.ironpanthers.frc2020.util.SteeringAdjuster;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToTarget extends CommandBase {
    private final Drive drive;
   
    private LimelightWrapper limelightWrapper;
    SteeringAdjuster steerer = SteeringAdjuster.getInstance();
    
    public TurnToTarget(Drive drive) {
        this.drive = drive;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        limelightWrapper = LimelightWrapper.getLimelightWrapperFront();
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        limelightWrapper.periodic();
        steerer.updateSteeringValues();
        drive.setOutputPercent(steerer.getLeftSteeringAdjust(), steerer.getRightSteeringAdjust());
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
