package com.ironpanthers.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Units;

// read in metric, publish in feet
public class Dashboard {

    private static Dashboard instance = null;

    public static Dashboard getInstance() {
        if (instance == null) {
            instance = new Dashboard();
        }
        return instance;
    }

    private final NetworkTableEntry robotX;
    private final NetworkTableEntry robotY;
    private final NetworkTableEntry robotHeading;

    private final NetworkTableEntry pathX;
    private final NetworkTableEntry pathY;
    private final NetworkTableEntry pathHeading;

    private final NetworkTableEntry followingPath;

    private Dashboard() {
        var table = NetworkTableInstance.getDefault().getTable("Live_Dashboard");
        robotX = table.getEntry("robotX");
        robotY = table.getEntry("robotY");
        robotHeading = table.getEntry("robotHeading");

        followingPath = table.getEntry("isFollowingPath");
        pathX = table.getEntry("pathX");
        pathY = table.getEntry("pathY");
        pathHeading = table.getEntry("pathHeading");

        publishPathFollowing(false);
    }

    public void publishRobotPose(Pose2d pose) {
        this.robotX.setDouble(Units.metersToFeet(
            pose.getTranslation().getX()
        ));
        this.robotY.setDouble(Units.metersToFeet(
            pose.getTranslation().getY()
        ));
        this.robotHeading.setDouble(pose.getRotation().getRadians());
    }

    public void publishPathSample(Pose2d pose) {
        this.pathX.setDouble(Units.metersToFeet(
            pose.getTranslation().getX()
        ));
        this.pathY.setDouble(Units.metersToFeet(
            pose.getTranslation().getY()
        ));
        this.pathHeading.setDouble(pose.getRotation().getRadians());
        publishPathFollowing(true);
    }

    public void publishPathFollowing(boolean bool) {
        this.followingPath.setBoolean(bool);
    }
}