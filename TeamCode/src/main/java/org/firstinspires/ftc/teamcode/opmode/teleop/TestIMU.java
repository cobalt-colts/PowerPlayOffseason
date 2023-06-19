package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@TeleOp
public class TestIMU extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap,telemetry,false);


        waitForStart();
        robot.drive.startIMUThread(this);

        while (opModeIsActive()){
            telemetry.addData("Current Heading: ", robot.drive.getAngle());
            telemetry.update();
        }
    }
}
