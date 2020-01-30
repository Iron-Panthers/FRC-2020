package com.ironpanthers.frc2020.util;

import java.util.ArrayList;

import com.ironpanthers.frc2020.Constants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    public double[] tcornxy;
    // public Number[] tcornx;
    // public Number[] tcorny;

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
        // tcornx = table.getEntry("tcornx").getNumberArray(new Number[1]);
        // tcorny = table.getEntry("tcorny").getNumberArray(new Number[1]);
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
    }public double getThor() {
        return thor;
    }public double getTs() {
        return ts;
    }
    public double getCamtran() {
        return camtran;
    }

    public double calculateHorizontalDistance() {
        double d = (Constants.Vision.HEIGHT_GROUND_TO_TARGET - Constants.Vision.HEIGHT_GROUND_TO_LIMELIGHT) / Math.tan((Constants.Vision.ANGLE_MOUNT_TO_LIMELIGHT + ty)*(Math.PI/180)); // ty: vertical offset angle in degrees
        return d;
    }

    public double calculateDiagonalDistance(){
        double d = Math.sqrt(Math.pow(calculateHorizontalDistance(),2) + Math.pow(Constants.Vision.HEIGHT_GROUND_TO_TARGET-Constants.Vision.HEIGHT_GROUND_TO_LIMELIGHT, 2));
        return d;
    }

    public double calcHoleOffset() {

        if(tcornxy.length != 8) {
            SmartDashboard.putNumber("tcornxy length", tcornxy.length);
            SmartDashboard.putNumber("four corners of the target cannot be used", 0);
            return 0.0;
        }

        // for(int i = 0; i < tcornxy.length; i++) {
        //     SmartDashboard.putNumber("tcornxy " + i, tcornxy[i]);
        // }

        double[] tcornx = {tcornxy[0], tcornxy[2], tcornxy[4], tcornxy[6]};
        double[] tcorny = {tcornxy[1], tcornxy[3], tcornxy[5], tcornxy[7]};

        double tleftx = 0; //x coord of top left corner of hexagon
        double tlefty = 0;
        double trightx = 0;
        double trighty = 0;
        double bleftx = 0;
        double blefty = 0;
        double brightx = 0;
        double brighty = 0;

        ArrayList<Double> orderedx = new ArrayList<Double>();
        ArrayList<Double> orderedy = new ArrayList<Double>();

        //keeps associated x and y values from tcornx and tcorny at the same index as each other in the new lists.
        //orders the list from lowest to highest x
        
        orderedx.add(tcornx[0]);
        orderedy.add(tcorny[0]);
        for (int i = 1; i < tcornx.length; i++) {
            int size = orderedx.size();
            for(int n = 0; n < size; n++) {
                if(tcornx[i] < orderedx.get(n)) {
                    orderedx.add(n, tcornx[i]);
                    orderedy.add(n, tcorny[i]);
                    break;
                }
                if(n==size-1) {
                    orderedx.add(tcornx[i]);
                    orderedy.add(tcorny[i]);
                }
            }
            
        }

        tleftx = orderedx.get(0);
        tlefty = orderedy.get(0);
        bleftx = orderedx.get(1);
        blefty = orderedy.get(1);
        trightx = orderedx.get(3);
        trighty = orderedy.get(3);
        brightx = orderedx.get(2);
        brighty = orderedy.get(2);

        /*if(orderedy.get(0) < orderedy.get(1)) {
            tleftx = orderedx.get(0);
            tlefty = orderedy.get(0);
            bleftx = orderedx.get(1);
            blefty = orderedy.get(1);
        } else {
            tleftx = orderedx.get(1);
            tlefty = orderedy.get(1);
            bleftx = orderedx.get(0);
            blefty = orderedy.get(0);
        }

        if(orderedy.get(2) < orderedy.get(3)) {
            trightx = orderedx.get(2);
            trighty = orderedy.get(2);
            brightx = orderedx.get(3);
            brighty = orderedy.get(3);
        } else {
            trightx = orderedx.get(3);
            trighty = orderedy.get(3);
            brightx = orderedx.get(2);
            brighty = orderedy.get(2);
        }*/

        SmartDashboard.putNumber("top left x: ", tleftx);
        SmartDashboard.putNumber("top left y: ", tlefty);
        SmartDashboard.putNumber("top right x: ", trightx);
        SmartDashboard.putNumber("top right y: ", trighty);
        SmartDashboard.putNumber("bottom left x: ", bleftx);
        SmartDashboard.putNumber("bottom left y: ", blefty);
        SmartDashboard.putNumber("bottom right x: ", brightx);
        SmartDashboard.putNumber("bottom right y: ", brighty);

        SmartDashboard.putNumber("diagonal distance from target: ", calculateDiagonalDistance());
        SmartDashboard.putNumber("horizontal distance from target: ", calculateHorizontalDistance());

        ThreeDimensionalSegment topLine = new ThreeDimensionalSegment(tleftx, tlefty, trightx, trighty, Constants.Vision.TOP_LINE_MAGNITUDE_AT_DISTANCE / calculateDiagonalDistance());
        //ThreeDimensionalSegment botLine = new ThreeDimensionalSegment(bleftx, blefty, brightx, brighty, Constants.Vision.BOTTOM_LINE_MAGNITUDE_AT_DISTANCE / calculateDiagonalDistance());

        //average of the estimated angles using the top and bottom lines
        double offsetAngle = topLine.getOffsetAngle( topLine.magnitude * Constants.Vision.OUTER_TO_HOLE_DISTANCE_PER_TL_LENGTH, calculateHorizontalDistance() );
        
        SmartDashboard.putNumber("hole offset angle: ", offsetAngle);

        double leftHeight = tlefty - blefty;
        double rightHeight = trighty - brighty;

        //copysign() to determine whether the adjust is going to be left or right
        return Math.copySign(offsetAngle, rightHeight - leftHeight);

    }
}