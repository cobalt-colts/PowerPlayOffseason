package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@TeleOp
public class Opmode extends CommandOpMode {
    private Robot robot;
    private ElapsedTime timer;

    private DigitalChannel receiver;

    private boolean prevTouchpad = false;
    private boolean currTouchpad = false;

    private boolean autoPickup = false;

    GamepadEx driverOp;
    GamepadEx toolOp;

    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();

        robot = new Robot(hardwareMap,telemetry,false);
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
            robot.startIMUThread(this);
        }
        toggleAuto();
        if(autoPickup && !gamepad1.left_bumper){
            if(robot.sensor.getState()){
                robot.intake.update(IntakeSubsystem.ClawState.OPEN);
            }else{
                robot.intake.update(IntakeSubsystem.ClawState.CLOSED);
            }
        }else if(autoPickup && (gamepad1.left_bumper || gamepad2.left_bumper)){
            robot.intake.update(IntakeSubsystem.ClawState.OPEN);
            sleep(600);
        }else{
            if(gamepad1.left_bumper || gamepad2.left_bumper) robot.intake.update(IntakeSubsystem.ClawState.OPEN);
            if(gamepad1.right_bumper || gamepad2.right_bumper) robot.intake.update(IntakeSubsystem.ClawState.CLOSED);
        }
        robot.read();
        //drivetrain
        robot.fieldRelative(driverOp.getLeftX(), driverOp.getLeftY(), driverOp.getRightX(), driverOp.getButton(GamepadKeys.Button.X));
        //turret
        robot.turret.setCurrentPower(0.8 * toolOp.getRightX());
        //vertical
        robot.vertical.setPower(toolOp.getLeftY());
        //horizontal
        robot.horizontal.setPos(Math.min(0.4,Math.max(0,robot.horizontal.getPos() + 0.005 * (toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)))));
        //wrist
        if(gamepad2.y) robot.intake.update(IntakeSubsystem.WristState.STOW);
        if(gamepad2.b) robot.intake.setWristPosition(0.3);
        if(gamepad2.a) robot.intake.update(IntakeSubsystem.WristState.ACTIVE);
        //update * write
        robot.loop();
        robot.write();
    }

    public void toggleAuto(){
        currTouchpad = gamepad1.touchpad;
        if(currTouchpad && !prevTouchpad) {
            autoPickup = !autoPickup;
        }
    }
}