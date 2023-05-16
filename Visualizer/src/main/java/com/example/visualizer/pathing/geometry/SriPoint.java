package com.example.visualizer.pathing.geometry;

import org.opencv.core.Point;

public class SriPoint extends Point {

    public int index;
    public double distanceInPath = 0;
    public double curvature = 0;
    public double targetVelocity = 0;

    public SriPoint(double x, double y, double distanceInPath){
        this(x,y);
        this.distanceInPath = distanceInPath;
    }

    public SriPoint(double x, double y){
        super(x,y);
    }

    public SriPoint(SriPoint other){
        this(other.x,other.y);
    }

    public SriPoint clone() {
        return new SriPoint(this);
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
