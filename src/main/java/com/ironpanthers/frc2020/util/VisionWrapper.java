package com.ironpanthers.frc2020.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionWrapper {

    public static NetworkTable table;
    public static double tx;
    public static double ty;
    public static double ta;
    public static double tv;
    public static double ts;
    public static double tvert;
    public static double thor;
    public static double x;
    public static double y;
    public static double v;
    public VisionWrapper() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx").getDouble(0.0);
        ty = table.getEntry("ty").getDouble(0.0);
        tv = table.getEntry("tv").getDouble(0.0);
        ta = table.getEntry("ta").getDouble(0.0);
        ts = table.getEntry("ts").getDouble(0.0);
        tvert = table.getEntry("tvert").getDouble(0.0);
        thor = table.getEntry("thor").getDouble(0.0);
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
    
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }public double getV() {
        return v;
    }public double getTvert() {
        return tvert;
    }public double getThor() {
        return thor;
    }public double getTs() {
        return ts;
    }
}