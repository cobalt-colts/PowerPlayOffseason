package org.firstinspires.ftc.teamcode.common.pathing.path;

import org.firstinspires.ftc.teamcode.common.pathing.geometry.SriPoint;

import org.opencv.core.Point;

import java.util.ArrayList;

//Heavily Copied from FRC 2137

public class SriFollower {
    ArrayList<SriPoint> path;
    double maxAcceleration;
    double lookaheadDistance;
    double trackWidth;
    double kv;
    double ka;
    double kp;

    double targetVelocity;
    double leftSpeed;
    double rightSpeed;

    int lastRateLimiterCallTime;

    SriPoint closestPoint;
    Point lookaheadPoint;
    SriPoint lookaheadPointSegmentStart;

    RobotPosition currentPosition;

    public SriFollower(SriPath sriPath, double maxAccel, double lookahead, double trackWidth, double kv, double ka, double kp){
        this.path = sriPath.getBeizerPath();
        this.maxAcceleration = maxAccel;
        this.lookaheadDistance = lookahead;
        this.trackWidth = trackWidth;
        this.kv = kv;
        this.ka = ka;
        this.kp = kp;

        closestPoint = path.get(0);
        lookaheadPoint = path.get(0);
        lookaheadPointSegmentStart = path.get(0);
    }

    //idk make this cmd based sometime
    public void update() {
        updateClosestPoint(closestPoint, currentPosition);
        updateLookaheadPoint(lookaheadPoint, lookaheadPointSegmentStart, currentPosition, lookaheadDistance);

        double relativeX = findLookaheadRelativeX(lookaheadPoint, currentPosition);
        double unsidedCurvature = 2 * Math.abs(relativeX) / (Math.pow(lookaheadDistance,2));
        double curvature = unsidedCurvature * Math.signum(relativeX);

        double oldTargetVelocity = targetVelocity;
        targetVelocity = findTargetVelocity(maxAcceleration, oldTargetVelocity, closestPoint);

        double[] wheelSpeeds = calculateWheelSpeeds(targetVelocity, curvature, trackWidth);
        leftSpeed = wheelSpeeds[0]; //set public variables to new values
        rightSpeed = wheelSpeeds[1];
        //calculate wheel speeds
    }

    public void updateRobotPosition(RobotPosition position) {
        currentPosition = position;
    }


    public boolean isFinished() {
        // return closestPoint == path.get(path.size()-1); // won't work, stopping will be wonky
        return findFractionalIndex(lookaheadPoint, lookaheadPointSegmentStart) > path.get(path.size()-1).index;
    }

    private void updateClosestPoint(SriPoint startingPoint, RobotPosition currentPosition){
        SriPoint currentClosest = startingPoint;
        //@TODO Add Index Code
        double currentClosestDistance = MathFunctions.findDistance(path.get(startingPoint.index), currentPosition);

        for (int i = startingPoint.index + 1; i < path.size(); i++) {
            SriPoint currentPoint = path.get(i);
            double currentDistance = MathFunctions.findDistance(currentPoint, currentPosition);
            if (currentDistance < currentClosestDistance) {
                currentClosest = path.get(i);
                currentClosestDistance = currentDistance;
            }
        }

        closestPoint = currentClosest;
    }

    private double findLookaheadRelativeX(Point lookaheadPoint, RobotPosition currentPosition) {
        double robotX = currentPosition.x;
        double robotY = currentPosition.y;
        double robotAngle = currentPosition.angle;

        double lookaheadX = lookaheadPoint.x;
        double lookaheadY = lookaheadPoint.y;

        //Math to fine point distance from robot line
        double a = -Math.tan(robotAngle);
        double b = 1;
        double c = (Math.tan(robotAngle) * robotX) - robotY;
        double unsidedX = Math.abs(a * lookaheadX + b * lookaheadY + c) / Math.sqrt(Math.pow(a,2) + Math.pow(b,2));

        //Now find side of the robot line X is on
        double side = Math.signum((Math.sin(robotAngle) * (lookaheadX  - robotX)) -
                (Math.cos(robotAngle) * (lookaheadY - robotY)));

        double sidedX = unsidedX * side;
        return sidedX;
    }

    private void updateLookaheadPoint(Point startingLookaheadPoint, SriPoint startingPreviousPoint,
                                      RobotPosition currentPosition, double lookaheadDistance) {

        //set to previous lookahead and prior point incase no new point is found
        SriPoint newLookaheadPreviousPoint = startingPreviousPoint;
        Point newLookahead = startingLookaheadPoint;
        double oldFractionalIndex = findFractionalIndex(startingLookaheadPoint, startingPreviousPoint);
        for (int i = startingPreviousPoint.index; i < path.size() - 1; i++) { // Point can't go backwards in segments
            SriPoint lineStart = path.get(i);
            SriPoint lineEnd = path.get(i+1);
            ArrayList<Point> intersections = MathFunctions.circleLineIntersection(currentPosition, lookaheadDistance,
                    lineStart, lineEnd);
            boolean toBreak = true;
            switch(intersections.size()) {
                case 0: // No intersections, skip this point
                    break;
                case 1: // 1 intersection, check if fractional index increases, no going backwards
                    if(findFractionalIndex(intersections.get(0), lineStart) > oldFractionalIndex) {
                        newLookahead = intersections.get(0);
                        newLookaheadPreviousPoint = lineStart;
                        toBreak = true;
                    }
                    break;
                case 2: // 2 intersections, choose first point where fractional index increases
                    double fractionalIndex0 = findFractionalIndex(intersections.get(0), lineStart);
                    double fractionalIndex1 = findFractionalIndex(intersections.get(1), lineStart);
                    if (fractionalIndex0 > fractionalIndex1 && fractionalIndex0 > oldFractionalIndex) {
                        newLookahead = intersections.get(0);
                        newLookaheadPreviousPoint = lineStart;
                        toBreak = true;
                    } else if (fractionalIndex1 > fractionalIndex0 && fractionalIndex1 > oldFractionalIndex) {
                        newLookahead = intersections.get(1);
                        newLookaheadPreviousPoint = lineStart;
                        toBreak = true;
                    }
                    break;
            }
            if (toBreak) {
                break;
            }
        }

        lookaheadPoint = newLookahead;
        lookaheadPointSegmentStart = newLookaheadPreviousPoint;
    }


    private double findFractionalIndex(Point pointToFind, SriPoint previousPoint) {
        double wholeSegmentLength = MathFunctions.findDistance(previousPoint, path.get(previousPoint.index + 1));
        double smallSegmentLength = MathFunctions.findDistance(previousPoint, pointToFind);
        double fraction = smallSegmentLength/wholeSegmentLength;
        double fractionalIndex = previousPoint.index + fraction;

        /**
         * Fractional index is as follows
         * Take the PathPoint before it on the path, this is the base number
         * Then, find the total distance between that point and the point following it on the path
         * Divide the distance from the Point to the PathPoint by the overall segment length
         * Then add that to the PathPoint before it's index
         * This is used so that you never go backwards in the path
         */

        return fractionalIndex;
    }

    private double findTargetVelocity(double maxAcceleration, double previousTargetVelocity, SriPoint closestPoint) {
        double currentTime = System.currentTimeMillis();
        double maxChange = ((currentTime - lastRateLimiterCallTime) / 1000) * maxAcceleration;
        double output = MathFunctions.clamp(closestPoint.targetVelocity - previousTargetVelocity, -maxChange, maxChange);

        return output;
    }

    private double[] calculateWheelSpeeds(double targetVelocity, double curvature, double trackWidth) {
        double leftSpeed = targetVelocity * (2 + (curvature * trackWidth)) / 2;
        double rightSpeed = targetVelocity * (2 - (curvature * trackWidth)) / 2;
        double[] speedArray = {leftSpeed, rightSpeed};

        return speedArray;
    }

    /**
     * @return the leftSpeed
     */
    public double getLeftSpeed() {
        return leftSpeed;
    }

    /**
     * @return the rightSpeed
     */
    public double getRightSpeed() {
        return rightSpeed;
    }
}
