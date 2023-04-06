package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class TestLinkage extends LinearOpMode {
    private Servo horSlide;

    public static double currPos = 0.0;

    @Override
    public void runOpMode(){
        horSlide = hardwareMap.get(Servo.class, "horLinkage");

        waitForStart();
        while(opModeIsActive()){
            horSlide.setPosition(currPos);
            telemetry.addData("Pos: ", currPos);
            telemetry.update();
        }
    }
}
