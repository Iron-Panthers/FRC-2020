package com.ironpanthers.frc2020.auto.commands;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import com.ironpanthers.frc2020.commands.drive.ReportingRAMSETECommand;
import com.ironpanthers.frc2020.subsystems.Drive;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TestAutonomous extends SequentialCommandGroup {
    /**
     * Create a new TestAutonomous, which drives along a trajectory defined by
     * PathWeaver-like JSON.
     * 
     * @param drive The instance of the Drive subsystem for the autonomous sequence
     *              to use.
     */
    public TestAutonomous(Drive drive) throws IOException {
            // Add commands to the group via `addCommands` to handle scheduling
            addCommands(new InstantCommand(drive::shiftHigh, drive),
            new ParallelDeadlineGroup(new WaitCommand(1.5), new RunCommand(() -> drive.setOutputVolts(4, 4))),
            new RunCommand(() -> drive.setOutputVolts(0, 0)));
    }
}