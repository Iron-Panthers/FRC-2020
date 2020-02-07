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
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestAutonomous extends SequentialCommandGroup {
        /**
         * Create a new TestAutonomous, which is simply a command which causes the robot
         * to drive along an 's' curve trajectory. This exists SOLELY for testing
         * purposes.
         * 
         * @param drive The instance of the Drive subsystem for the autonomous sequence
         *              to use.
         */
        public TestAutonomous(Drive drive) {
                // Create trajectory config
                // Max velocity: 14 fs-1
                // Max acceleration 2 fs-2
                var config = new TrajectoryConfig(Units.feetToMeters(14.0), Units.feetToMeters(2.0))
                                .setKinematics(drive.kinematics());

                // Generate the trajectory
                var trajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(3, 0, new Rotation2d(0)), config);

                // Alias of trajectory-tracking command for readability
                var initialDriveCommand = new ReportingRAMSETECommand(trajectory, drive::currentPose,
                                new RamseteController(2, 0.7), drive.ff(), drive.kinematics(), drive::speeds,
                                drive.leftPIDController(), drive.rightPIDController(), drive::setOutputVolts, drive);

                // Add commands to the group via `addCommands` to handle scheduling
                addCommands(initialDriveCommand, new RunCommand(() -> drive.setOutputVolts(0, 0)));
        }
}