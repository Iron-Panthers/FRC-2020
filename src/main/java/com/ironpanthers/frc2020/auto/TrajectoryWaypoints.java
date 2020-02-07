package com.ironpanthers.frc2020.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class TrajectoryWaypoints {
    private TrajectoryWaypoints() {
        throw new UnsupportedOperationException("TrajectoryWaypoints is a utility class");
    }

    public static final Pose2d kGoalSideStartingPosition = new Pose2d(3.7086095340755274, 6.7506862003780705,
            new Rotation2d(0.2472970332740446));
}