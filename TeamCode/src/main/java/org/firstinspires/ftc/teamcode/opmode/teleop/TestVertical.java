package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@Config
@TeleOp
public class TestVertical extends LinearOpMode {

    private Robot robot;
    public static int ticks = 0;
    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap,telemetry,true);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while(opModeIsActive()){
            robot.read();
            robot.vertical.setTargetPos(ticks);

            telemetry.addData("Goal: ", ticks);
            telemetry.addData("Current: ", robot.vertical.getPos());
            telemetry.update();
            robot.loop();
            robot.write();
        }
    }
}
