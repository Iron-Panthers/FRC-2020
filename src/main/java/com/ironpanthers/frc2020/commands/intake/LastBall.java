/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.intake;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.util.Util;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LastBall extends CommandBase {
  /**
   * Creates a new LastBall.
   */
  ConveyorBelt conveyor;
  int targetPosition;
  public LastBall(ConveyorBelt conveyor) {
    this.conveyor = conveyor;
    addRequirements(conveyor);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPosition = conveyor.getPosition() - Constants.Conveyor.kShiftEncoderDistanceLast;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conveyor.setPosition(targetPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Util.epsilonEquals(conveyor.getPosition(), targetPosition, Constants.Conveyor.kPositionErrorTolerance);
  }
}
