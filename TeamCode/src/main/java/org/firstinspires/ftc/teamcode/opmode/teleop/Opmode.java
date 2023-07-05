package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.HorizontalPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.TurretPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.VerticalPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@TeleOp
@Config
public class Opmode extends LinearOpMode {
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

    private boolean macro = false;
    public static int turretPos = 1000;
    public static int slidePos = 1500;
    public static double horPos = 0.25;
    GamepadEx driverOp;
    GamepadEx toolOp;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        robot = new Robot(hardwareMap, telemetry, false);
        guide = hardwareMap.get(Servo.class, "guide");
        robot.reset();
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);


        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            if (timer == null) {
                timer = new ElapsedTime();
                robot.reset();
                robot.drive.startIMUThread(this);
                robot.drive.lockedTarget = robot.drive.getAngle();
            }
            telemetry.addData("Locked Target", robot.drive.lockedTarget);
            telemetry.addData("Angle", robot.drive.getAngle());
            robot.read();

            if (gamepad2.dpad_left && !macro) {
                macro = true;
            }

            while(opModeIsActive()) {
                if (macro) {

                    telemetry.addData("Auto", macro);
                    robot.vertical.setTargetPos(slidePos);
                    if(robot.vertical.getPos() < 500) break;
                    robot.turret.setTargetPos(turretPos);
                    robot.horizontal.setPos(horPos);
                    robot.intake.update(IntakeSubsystem.WristState.SLIGHT);

                    if(Math.abs(robot.turret.getPos() - turretPos) > 20 && Math.abs(robot.vertical.getPos() - slidePos) > 20) break;
                    macro = false;

                } else {
                    updateRobot();
                }
                break;
            }

            robot.loop(macro);
            robot.write();

            telemetry.addData("Vertical Goal", robot.vertical.getTargetPosition());
            telemetry.addData("Vertical Curr", robot.vertical.getPos());
            telemetry.addData("Vertical Command Scheduled", CommandScheduler.getInstance().isScheduled(new VerticalPositionCommand(robot.vertical, slidePos, 30, 5)));
            telemetry.update();
        }
    }

    public void updateRobot(){
        //intake
        robot.updateClaw(gamepad1, gamepad2);
        currY = gamepad1.y;

        //drivetrain
        if (currY && !prevY) {
            locked = !locked;
            if (locked) robot.drive.lockedTarget = robot.drive.getAngle();
        }

        if (locked) {
            robot.drive.lockedFieldRelative(driverOp.getLeftX(), driverOp.getLeftY(), driverOp.getRightX());
        } else {
            robot.drive.fieldRelative(driverOp.getLeftX(), driverOp.getLeftY(), driverOp.getRightX(), driverOp.getButton(GamepadKeys.Button.X));
        }

        if (CommandScheduler.getInstance().isScheduled())

            //turret
            robot.turret.setCurrentPower(0.8 * toolOp.getRightX());
        //vertical
        robot.vertical.setPower(toolOp.getLeftY());
        //horizontal modifiers
        outModifier = Math.min(0.005, 0.005 - (robot.horizontal.getPos() / 100));
        inModifier = Math.min(0.005, (robot.horizontal.getPos() / 80));
        //horizontal
        robot.horizontal.setPos(Math.min(0.4, Math.max(0, robot.horizontal.getPos() + (outModifier * toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - inModifier * (toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - 0.002)))));
        //wrist


        if (gamepad2.y) robot.intake.update(IntakeSubsystem.WristState.STOW);
        if (gamepad2.b) robot.intake.update(IntakeSubsystem.WristState.SLIGHT);
        if (gamepad2.a) robot.intake.update(IntakeSubsystem.WristState.ACTIVE);
        //update * write

        if(gamepad1.dpad_left) guide.setPosition(0);
        if(gamepad1.dpad_right) guide.setPosition(1);

        telemetry.addData("Locked", locked);

    }

}