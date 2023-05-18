package org.firstinspires.ftc.teamcode.common.pathing.geometry;

import org.opencv.core.Point;

public class SriPoint extends Point {

    public int index;
    public double distanceInPath = 0;
    public double curvature = 0;
    public double targetVelocity = 0;


    public SriPoint(double x, double y, int index){
        super(x,y);
        this.index = index;
    }

    public SriPoint(double x, double y){
        super(x,y);
    }




    public void setDistanceInPath(double distanceInPath) {
        this.distanceInPath = distanceInPath;
    }


    public void setCurvature(double curvature) {
        this.curvature = curvature;
    }


    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public double distance(SriPoint other) {
        return Math.hypot(other.x - this.x, other.y - this.y);
    }

    public double angleTo(SriPoint other) {
        return Math.atan2(other.y - this.y, other.x - this.x);
    }

}
