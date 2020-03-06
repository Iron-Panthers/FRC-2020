package com.ironpanthers.frc2020.util;


import com.ironpanthers.frc2020.Constants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightWrapper {
    private final NetworkTable table;
    private double camtran, tx, ty, ta, tv, ts, tvert, thor;
    private double[] tcornxy;

    private static LimelightWrapper frontLimelight = new LimelightWrapper();

    public LimelightWrapper() {
        table = NetworkTableInstance.getDefault().getTable(Constants.Vision.kLimelightName);
        periodic();
    }

    public static LimelightWrapper getLimelightWrapperFront() {
        return frontLimelight;
    }

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
        if (table.getEntry("ledMode").getDouble(0.0) == 1) {
            setLightMode(LightMode.ON);
        }
    }

    /**
     * set the light on, off, blink, or to follow the pipeline
     */
    public void setLightMode(LightMode mode) {
        switch(mode) {
            case PIPELINE:
                table.getEntry("ledMode").setNumber(0);
                break;
            case ON:
                table.getEntry("ledMode").setNumber(3);
                break;
            case BLINK:
                table.getEntry("ledMode").setNumber(2);
                break;
            case OFF:
                table.getEntry("ledMode").setNumber(1);
                break;
            default:
                table.getEntry("ledMode").setNumber(0);
        }
    }

    /**
     * it should be noted that this does not return in terms of pixels. Docs say: 
     * "Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)"
     * @return
     */
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
        SmartDashboard.putBoolean("target visible", visible);
        return visible;
    }
}