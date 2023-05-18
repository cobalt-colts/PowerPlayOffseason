package org.firstinspires.ftc.teamcode.common.pathing.path;

import org.firstinspires.ftc.teamcode.common.pathing.geometry.SriPoint;

import java.util.ArrayList;

public class SriPath {

    //waypoints
    private SriPoint p1,p2,p3,p4;


    //path
    private ArrayList<SriPoint> beizerPath = new ArrayList<>();

    public SriPath(SriPoint p1, SriPoint p2, SriPoint p3, SriPoint p4, double MAX_VELOCITY, double MAX_ACCELERATION){
        this.p1= p1;
        this.p2= p2;
        this.p3= p3;
        this.p4= p4;

        generateBeizer(MAX_VELOCITY,MAX_ACCELERATION);
    }
    //x: ( (1-t) * ( (1-t) * ( (1-t)*p1.x + t * p2.x) + t * ( (1-t) * p2.x + t * p3.x)) + t * ( (1-t) * ((1-t) * p2.x + t * p3.x) + t * ( (1-t) * p3.x + t * p4.x))
    public void generateBeizer(double MAX_VELOCITY, double MAX_ACCELERATION){


        double distance = 0;

        int idx = 0;


        for(double t = 0; t <= 1; t+= 0.02){
            //Beizer Parametric Points
            double x = (1-t) * ( (1-t) * ( (1-t)*p1.x + t * p2.x) + t * ( (1-t) * p2.x + t * p3.x)) + t * ( (1-t) * ((1-t) * p2.x + t * p3.x) + t * ( (1-t) * p3.x + t * p4.x));
            double y = (1-t) * ( (1-t) * ( (1-t)*p1.y + t * p2.y) + t * ( (1-t) * p2.y + t * p3.y)) + t * ( (1-t) * ((1-t) * p2.y + t * p3.y) + t * ( (1-t) * p3.y + t * p4.y));
            beizerPath.add(new SriPoint(x,y,idx));
            idx++;

            if(t == 0) continue;
            distance += beizerPath.get(beizerPath.size() - 1).distance(beizerPath.get(beizerPath.size()-2));
            beizerPath.get(beizerPath.size() - 1).setDistanceInPath(distance);

        }

        double[] uncorrectedVelocities = new double[beizerPath.size()];
        uncorrectedVelocities[0] = 0; uncorrectedVelocities[beizerPath.size() - 1] = 0;

        double[] correctedVelocities = new double[beizerPath.size()];

        for(int i=1; i < beizerPath.size() - 1; i++){
            double x1 = beizerPath.get(i-1).x + 0.001; //prevent divide by 0
            double x2 = beizerPath.get(i).x;
            double x3 = beizerPath.get(i+1).x;

            double y1 = beizerPath.get(i-1).y;
            double y2 = beizerPath.get(i).y;
            double y3 = beizerPath.get(i+1).y;

            double k1 = 0.5 * (x1*x1 + y1*y1 -x2*x2 - y2*y2)/(x1-x2);
            double k2 = (y1-y2)/(x1-x2);

            double b = 0.5 * (x2*x2 - 2 * x2 * k1 + y2*y2 - x3*x3 + 2 * x3 * k1 - y3*2) / (x3 * k2 - y3 + y2 - x2 * k2);
            double a = k1 - k2 * b;

            double r = Math.sqrt((x1-a) * (x1-a) + (y1 - b) * (y1 - b));

            double curvature = (r == 0) ? 0 : 1.0 / r;
            //r -> infinity?

            beizerPath.get(i).setCurvature(curvature);
            uncorrectedVelocities[i] = Math.min(MAX_VELOCITY, 2 / curvature);

        }
        correctedVelocities[correctedVelocities.length - 1] = 0; //stop at end

        for(int i = beizerPath.size() - 2; i >= 0; i--){
            double d = beizerPath.get(i).distance(beizerPath.get(i+1));

            correctedVelocities[i] = Math.min(uncorrectedVelocities[i], Math.sqrt(correctedVelocities[i+1] + 2 * MAX_ACCELERATION * d));
            beizerPath.get(i).setTargetVelocity(correctedVelocities[i]);
        }
    }

    public ArrayList<SriPoint> getBeizerPath() {
        return beizerPath;
    }
}
