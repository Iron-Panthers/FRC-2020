package com.ironpanthers.frc2020.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionWrapper {

    public static NetworkTable table;
    public static NetworkTableEntry tx;
    public static NetworkTableEntry ty;
    public static NetworkTableEntry ta;
    public static NetworkTableEntry tv;
    public static double x;
    public static double y;
    public static double v;

    public VisionWrapper() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        tv = table.getEntry("tv");
        ta = table.getEntry("ta");
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        v = tv.getDouble(0.0);
    } 

    public NetworkTableEntry getTableX() {
        return tx;
    }
    public NetworkTableEntry getTableY() {
        return ty;
    }
    public NetworkTableEntry getTableV() {
        return tv;
    }
    public NetworkTableEntry getTableA() {
        return ta;
    }
    
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }public double getV() {
        return v;
    }
}