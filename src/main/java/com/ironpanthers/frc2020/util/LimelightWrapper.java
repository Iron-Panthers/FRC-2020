package com.ironpanthers.frc2020.util;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightWrapper {
    private final NetworkTable table;
    private double camtran, tx, ty, ta, tv, ts, tvert, thor;
    private double[] tcornxy;

    private static LimelightWrapper frontLimelight = new LimelightWrapper();

    public LimelightWrapper() {
        table = NetworkTableInstance.getDefault().getTable("limelight-b");
        periodic();
    }

    public static LimelightWrapper getLimelightWrapperFront() {
        return frontLimelight;
    }

    /**
     * This method must be run periodically to refresh the values read by the
     * Limelight.
     * <p>
     * TODO This structure implies that subsystems may actually be the desired
     * "model" to use for the LimelightWrapper class. This task is open
     */
    public void periodic() {
        tx = table.getEntry("tx").getDouble(0.0);
        ty = table.getEntry("ty").getDouble(0.0);
        tv = table.getEntry("tv").getDouble(0.0);
        ta = table.getEntry("ta").getDouble(0.0);
        ts = table.getEntry("ts").getDouble(0.0);
        camtran = table.getEntry("camtran").getDouble(0);
        tvert = table.getEntry("tvert").getDouble(0.0);
        thor = table.getEntry("thor").getDouble(0.0);
        tcornxy = table.getEntry("tcornxy").getDoubleArray(new double[1]);
    }

    public double getTableX() {
        return tx;
    }

    public double getTableY() {
        return ty;
    }

    public double getTableV() {
        return tv;
    }

    public double getTableA() {
        return ta;
    }

    public double getTvert() {
        return tvert;
    }

    public double getThor() {
        return thor;
    }

    public double getTs() {
        return ts;
    }

    public double getCamtran() {
        return camtran;
    }

    public double[] getTCornXY() {
        return tcornxy;
    }

    public boolean targetVisible() {
        boolean visible = getTCornXY().length == 8 && getTableV() == 1;
        if(!visible) {
            SmartDashboard.putNumber("can't see target or enough of the corners", 0);
        }
        return visible;
    }
}