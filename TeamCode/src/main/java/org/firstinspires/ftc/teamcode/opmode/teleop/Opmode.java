package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@TeleOp
public class Opmode extends CommandOpMode {
    private Robot robot;
    private Servo guide;
    private ElapsedTime timer;

    private DigitalChannel receiver;

    private double outModifier = 0.0;
    private double inModifier = 0.0;

    private static double guideStowLeft = 0;
    private static double guideActiveLeft = 0.4;
    private static double guideStowRight = 0.6;
    private static double guideActiveRight = 1;

    private boolean prevY = false;
    private boolean currY = false;
    private boolean locked = false;

    GamepadEx driverOp;
    GamepadEx toolOp;

    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();

        robot = new Robot(hardwareMap,telemetry,false);
        guide = hardwareMap.get(Servo.class, "guide");
        robot.reset();
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),this.telemetry);




        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);
    }


    @Override
    public void run(){
        if (timer == null) {
            timer = new ElapsedTime();
            robot.reset();
            robot.drive.startIMUThread(this);
            robot.drive.lockedTarget = robot.drive.getAngle();
        }
        telemetry.addData("Locked Target", robot.drive.lockedTarget);
        telemetry.addData("Angle", robot.drive.getAngle());
        robot.read();
        //intake

        robot.updateClaw(gamepad1,gamepad2);
        currY = gamepad1.y;

        //drivetrain
        if(currY && !prevY){
            locked = !locked;
            if(locked) robot.drive.lockedTarget = robot.drive.getAngle();
        }

        if(locked) {
            robot.drive.lockedFieldRelative(driverOp.getLeftX(), driverOp.getLeftY(), driverOp.getRightX());
        }else {
            robot.drive.fieldRelative(driverOp.getLeftX(), driverOp.getLeftY(), driverOp.getRightX(), driverOp.getButton(GamepadKeys.Button.X));
        }


        //turret
        robot.turret.setCurrentPower(0.8 * toolOp.getRightX());
        //vertical
        robot.vertical.setPower(toolOp.getLeftY());
        //horizontal modifiers
        outModifier = Math.min(0.005, 0.005 - (robot.horizontal.getPos() / 100));
        inModifier = Math.min(0.005, (robot.horizontal.getPos() / 80));
        //horizontal
        robot.horizontal.setPos(Math.min(0.4,Math.max(0,robot.horizontal.getPos() + (outModifier * toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - inModifier * (toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - 0.002)))));
        //wrist



        if(gamepad2.y) robot.intake.update(IntakeSubsystem.WristState.STOW);
        if(gamepad2.b) robot.intake.update(IntakeSubsystem.WristState.SLIGHT);
        if(gamepad2.a) robot.intake.update(IntakeSubsystem.WristState.ACTIVE);
        //update * write
        robot.loop();
        robot.write();

        telemetry.addData("Locked", locked);
        telemetry.update();
    }


}