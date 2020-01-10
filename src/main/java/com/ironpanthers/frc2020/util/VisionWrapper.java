package com.ironpanthers.frc2020.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionWrapper {

    public NetworkTable table;
    public double camtran;
    public double tx;
    public double ty;
    public double ta;
    public double tv;
    public double ts;
    public double tvert;
    public double thor;
    public double x;
    public double y;
    public double v;
    public VisionWrapper() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        initialize();
    } 

    public void initialize() {
        tx = table.getEntry("tx").getDouble(0.0);
        ty = table.getEntry("ty").getDouble(0.0);
        tv = table.getEntry("tv").getDouble(0.0);
        ta = table.getEntry("ta").getDouble(0.0);
        ts = table.getEntry("ts").getDouble(0.0);
        camtran = table.getEntry("camtran").getDouble(0);
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
    public double getCamtran() {
        return camtran;
    }
}