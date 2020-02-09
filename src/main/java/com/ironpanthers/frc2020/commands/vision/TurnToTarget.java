/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.vision;

import java.util.function.BooleanSupplier;

import com.ironpanthers.frc2020.subsystems.Drive;
import com.ironpanthers.frc2020.util.SteeringAdjuster;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToTarget extends CommandBase {
    private final Drive drive;
   
    SteeringAdjuster steerer;

    BooleanSupplier seeTarget;
    
    public TurnToTarget(Drive drive, SteeringAdjuster steerer, BooleanSupplier seeTarget) {
        this.drive = drive;
        this.steerer = steerer;
        this.seeTarget = seeTarget;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(seeTarget.getAsBoolean()) {
            steerer.updateSteeringValues();
            drive.setOutputPercent(steerer.getLeftSteeringAdjust(), steerer.getRightSteeringAdjust());
        } else {
            drive.setOutputPercent(0.0, 0.0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.setOutputPercent(0.0, 0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
