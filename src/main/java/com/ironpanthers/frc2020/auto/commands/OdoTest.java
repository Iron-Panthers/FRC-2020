/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.auto.commands;

import java.util.List;

import com.ironpanthers.frc2020.commands.drive.ReportingRAMSETECommand;
import com.ironpanthers.frc2020.subsystems.Drive;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OdoTest extends SequentialCommandGroup {
  public OdoTest(final Drive drive) {
    var turn = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), 
      List.of(new Translation2d(0.5, 0.5), new Translation2d(0.75, 0.75), new Translation2d (1, 1)),
      new Pose2d(1.5, 1.5, Rotation2d.fromDegrees(0)),
      new TrajectoryConfig(2, 2)
    );

    addCommands(
      new InstantCommand(drive::shiftHigh, drive),
      new ReportingRAMSETECommand(turn, drive::getCurrentPose, new RamseteController(), drive.getFeedforward(), drive.getKinematics(), drive::getWheelSpeeds, drive.getLeftPIDController(), drive.getRightPIDController(), drive::setOutputVolts, drive)
    );
  }
}
