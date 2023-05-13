package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class TestModule extends LinearOpMode {
    private DcMotor motor;
    private Servo servo;

    @Override
    public void runOpMode(){
        motor = hardwareMap.get(DcMotor.class, "motor");
        servo = hardwareMap.get(Servo.class, "servo");

        waitForStart();
        while (opModeIsActive()){
            motor.setPower(-gamepad1.left_stick_y);
            servo.setPosition(gamepad1.right_stick_x);
        }
    }
}
