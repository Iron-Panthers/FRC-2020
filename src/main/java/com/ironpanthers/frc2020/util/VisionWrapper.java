package com.ironpanthers.frc2020.util;

import java.util.ArrayList;

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
    public Number[] tcornx;
    public Number[] tcorny;

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
        tcornx = table.getEntry("tcornx").getNumberArray(new Number[1]);
        tcorny = table.getEntry("tcorny").getNumberArray(new Number[1]);

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
        double tleftx = 0; //x coord of top left corner of hexagon
        double tlefty = 0;
        double trightx = 0;
        double trighty = 0;
        double bleftx = 0;
        double blefty = 0;
        double brightx = 0;
        double brighty = 0;

        double[] orderedx = new double[4];
        double[] orderedy = new double[4];

        //keeps associated x and y values from tcornx and tcorn y at the same index as each other in the new lists.
        //orders the list from lowest to highest x
        for (int i = 0; i < tcornx.length; i++) {
            for(int n = 0; n < orderedx.length; n++) {
                if( (double) tcornx[i] < orderedx[n] || orderedx[n] == 0.0) {
                    for(int j = n; j < orderedx.length; j++) {
                        orderedx[j+1] = orderedx[j];
                        orderedy[j+1] = orderedx[j];
                    }
                    orderedx[n] = (double) tcornx[i];
                    orderedy[n] = (double) tcorny[i];
                    break;
                }
            }
        }

        if(orderedy[0] > orderedy[1]) {
            tleftx = orderedx[0];
            tlefty = orderedy[0];
            bleftx = orderedx[1];
            bleftx = orderedy[1];
        } else {
            tleftx = orderedx[1];
            tlefty = orderedy[1];
            bleftx = orderedx[0];
            blefty = orderedy[0];
        }

        if(orderedy[2] > orderedy[3]) {
            trightx = orderedx[2];
            trighty = orderedy[2];
            brightx = orderedx[3];
            brightx = orderedy[3];
        } else {
            trightx = orderedx[3];
            trighty = orderedy[3];
            brightx = orderedx[2];
            brightx = orderedy[2];
        }

        ThreeDimensionalSegment topLine = new ThreeDimensionalSegment(tleftx, tlefty, trightx, trighty, Constants.Vision.TOP_LINE_MAGNITUDE_AT_DISTANCE / calculateDiagonalDistance());
        ThreeDimensionalSegment botLine = new ThreeDimensionalSegment(bleftx, blefty, brightx, brighty, Constants.Vision.BOTTOM_LINE_MAGNITUDE_AT_DISTANCE / calculateDiagonalDistance());

        //average of the estimated angles using the top and bottom lines
        double offsetAngle = ( topLine.getOffsetAngle( topLine.magnitude * Constants.Vision.OUTER_TO_HOLE_DISTANCE_PER_TL_LENGTH, calculateHorizontalDistance()) + 
        botLine.getOffsetAngle(botLine.magnitude * Constants.Vision.OUTER_TO_HOLE_DISTANCE_PER_BL_LENGTH, calculateHorizontalDistance()) ) / 2;
        
        double leftHeight = tlefty - blefty;
        double rightHeight = trighty - brighty;

        //copysign() to determine whether the adjust is going to be left or right
        return Math.copySign( Math.sin(offsetAngle) * Constants.Vision.X_ADJUST_PER_DEGREE / calculateDiagonalDistance(), leftHeight - rightHeight );

    }
}