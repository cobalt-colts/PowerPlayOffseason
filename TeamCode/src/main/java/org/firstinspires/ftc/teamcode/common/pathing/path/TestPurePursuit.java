package org.firstinspires.ftc.teamcode.common.pathing.path;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.pathing.geometry.SriPoint;

import java.util.List;

@Config
@Autonomous
public class TestPurePursuit extends LinearOpMode {
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public static double kV, kA, kP;

    private List<Integer> lastEncPositions, lastEncVels;

    SriPath sriPath;
    SriFollower sriFollower;
    @Override
    public void runOpMode(){
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        StandardTrackingWheelLocalizer odometry = new StandardTrackingWheelLocalizer(hardwareMap,lastEncPositions,lastEncVels);


        sriPath = new SriPath(new SriPoint(0,0), new SriPoint(5,5), new SriPoint(0,25), new SriPoint(0,30), 20, 20);
        sriFollower = new SriFollower(sriPath,20,12,18,1,1,1);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);

        waitForStart();

        while(opModeIsActive()){
            if(sriFollower.isFinished()) break;

            RobotPosition currPos = new RobotPosition(odometry.getPoseEstimate().getX(),odometry.getPoseEstimate().getY(),odometry.getPoseEstimate().getHeading());
            sriFollower.updateRobotPosition(currPos);
            sriFollower.update();

            double leftFF = kV * sriFollower.leftSpeed + kA * 20; //maxAccel
            double rightFF = kV * sriFollower.rightSpeed + kA * 20;

            double leftFB = kP * (sriFollower.leftSpeed - odometry.getWheelVelocities().get(0));
            double rightFB = kP * (sriFollower.rightSpeed - odometry.getWheelVelocities().get(1));

            double leftPower = leftFF + leftFB;
            double rightPower = rightFF + rightFB;

            leftFront.setPower(leftPower);
            leftRear.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightRear.setPower(rightPower);

            telemetry.addData("Left Target Velocity: ", sriFollower.leftSpeed);
            telemetry.addData("Right Target Velocity: ", sriFollower.rightSpeed);

            telemetry.addData("Left Measured Velocity: ", odometry.getWheelVelocities().get(0));
            telemetry.addData("Right Measured Velocity: ", odometry.getWheelVelocities().get(1));


        }

    }
}
