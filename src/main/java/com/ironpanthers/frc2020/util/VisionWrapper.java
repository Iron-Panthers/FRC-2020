package com.ironpanthers.frc2020.util;

import com.ironpanthers.frc2020.Constants;

import edu.wpi.first.networktables.NetworkTable;
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
        loadVariables();
    } 

    public void loadVariables() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
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

    public double calculateHorizontalDistance() {
        double d = (Constants.HEIGHT_GROUND_TO_TARGET - Constants.HEIGHT_GROUND_TO_LIMELIGHT) / Math.tan((Constants.ANGLE_MOUNT_TO_LIMELIGHT*(Math.PI/180)) + (ty*(Math.PI/180))); // ty: vertical offset angle in degrees
        System.out.println("Horizontal distance: " + d);
        return d;
      }
    
      public double calculateDiagonalDistance(){
        double d = Math.sqrt(Math.pow(calculateHorizontalDistance(),2) + Math.pow(Constants.HEIGHT_GROUND_TO_TARGET-Constants.HEIGHT_GROUND_TO_LIMELIGHT, 2));
        System.out.println("Diagonal distance: " + d);
        return d;
      }
}