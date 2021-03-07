// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ironpanthers.frc2020.commands.drive;

import com.ironpanthers.frc2020.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnFine extends CommandBase {
  private final Drive drive;
  private final char direction;

  /** Creates a new TurnFine. */
  public TurnFine(Drive drive, char direction) {
    this.drive = drive;
    this.direction = direction;
    this.addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (direction == 'L') {
      drive.setOutputPercent(0.075, -0.075);
    } else {
      drive.setOutputPercent(-0.075, 0.075);
    }

    // TODO: implement run logic
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
