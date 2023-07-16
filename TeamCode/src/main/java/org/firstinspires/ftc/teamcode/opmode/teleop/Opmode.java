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
    private ElapsedTime timer;
    private ElapsedTime servoTimer;
    private boolean servoWait = false;
    private DigitalChannel receiver;

    private double guideX = 0;

    private boolean prevY = false;
    private boolean currY = false;
    private boolean locked = false;

    boolean prevGuide = false;
    boolean slow = false;

    private boolean macro = false;
    public static int turretPos = -400;
    public static int slidePos = 1600;
    public static double horPos = 0.0;

    public static int backPos = -1600;
    GamepadEx driverOp;
    GamepadEx toolOp;

    enum Macro{
        LEFT_HIGH,
        FRONT,
        BACK,
        RIGHT_HIGH,
        FINISHED,
        OFF
    }

    Macro macroState = Macro.OFF;
    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        robot = new Robot(hardwareMap, telemetry, false);

        robot.reset();
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);

        robot.horizontal.horSlide.setPosition(0.5);

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

            switch(macroState){
                case OFF:
                    updateRobot();
                    if (gamepad2.dpad_up) macroState = Macro.LEFT_HIGH;
                    else if(gamepad2.dpad_down) macroState = Macro.BACK;
                    break;
                case LEFT_HIGH:
                    macro = true;
                    telemetry.addData("Auto: ", macroState.toString());
                    robot.vertical.setTargetPos(1517);
                    robot.turret.setTargetPos(robot.turret.getPos());
                    robot.intake.update(IntakeSubsystem.WristState.SLIGHT);
                    if(Math.abs(robot.turret.getPos() - turretPos) > 20 && Math.abs(robot.vertical.getPos() - slidePos) > 20) break;
                    macroState = Macro.FINISHED;
                    break;
                case RIGHT_HIGH:
                    macro = true;
                    telemetry.addData("Auto: ", macroState.toString());
                    robot.horizontal.setPos(0);
                    robot.vertical.setTargetPos(slidePos);
                    if(robot.vertical.getPos() < 500) break;
                    robot.turret.setTargetPos(-1 * turretPos);
                    robot.horizontal.setPos(horPos);
                    robot.intake.update(IntakeSubsystem.WristState.SLIGHT);
                    if(Math.abs(robot.turret.getPos() - (-1*turretPos)) > 20 && Math.abs(robot.vertical.getPos() - slidePos) > 20) break;
                    robot.horizontal.setPos(horPos);
                    macroState = Macro.FINISHED;
                    break;
                case FRONT:

                    macro = true;
                    telemetry.addData("Auto: ", macroState.toString());
                    robot.intake.update(IntakeSubsystem.WristState.SLIGHT);
                    robot.horizontal.setPos(0.05);
                    robot.vertical.setTargetPos(100);
                    robot.turret.setTargetPos(0);
                    if(Math.abs(robot.turret.getPos()) > 20 && Math.abs(robot.vertical.getPos() - 100) > 20) break;
                    robot.intake.update(IntakeSubsystem.WristState.ACTIVE);
                    macroState = Macro.FINISHED;
                    break;
                case BACK:
                    macro = true;
                    telemetry.addData("Auto: ", macroState.toString());
                    robot.vertical.setTargetPos(73);
                    robot.turret.setTargetPos(robot.turret.getPos());
                    if(Math.abs(robot.turret.getPos() - backPos) > 20 && Math.abs(robot.vertical.getPos() - 100) > 20) break;
                    robot.intake.update(IntakeSubsystem.WristState.ACTIVE);
                    macroState = Macro.FINISHED;
                    break;
                case FINISHED:
                    macro = false;
                    macroState = Macro.OFF;
                    break;
            }

            if(gamepad2.left_stick_button) macroState = Macro.FINISHED;
            robot.loop(macro);
            robot.write();

            telemetry.addData("Vertical Goal", robot.vertical.getTargetPosition());
            telemetry.addData("Vertical Curr", robot.vertical.getPos());
            telemetry.addData("Turret Goal", robot.turret.getTargetPosition());
            telemetry.addData("Turret Curr", robot.turret.getPos());
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
            robot.drive.lockedFieldRelative(driverOp.getLeftX(), driverOp.getLeftY(), driverOp.getRightX(),telemetry);
        } else {
            robot.drive.fieldRelative(driverOp.getLeftX(), driverOp.getLeftY(), driverOp.getRightX(), driverOp.getButton(GamepadKeys.Button.X));
        }

        double horPos =
                (!gamepad2.touchpad_finger_1 ? Math.min(0.4, Math.max(0, robot.horizontal.getPos() + 0.02 *
                toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - 0.02*toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)))
                : (gamepad2.touchpad_finger_1_x + 1)/ 5.0);


        //turret
        double turretPower = (-0.5/0.4)*horPos + 0.8;
        robot.turret.setCurrentPower(turretPower * toolOp.getRightX());
        //vertical
        robot.vertical.setPower(toolOp.getLeftY());

        //horizontal

        if(gamepad2.guide && !prevGuide){
            slow = !slow;
        }
        robot.horizontal.setPos(slow ? 0.5 * horPos : horPos);
        //wrist


        if (gamepad2.y) robot.intake.update(IntakeSubsystem.WristState.STOW);
        if (gamepad2.b) robot.intake.update(IntakeSubsystem.WristState.SLIGHT);
        if (gamepad2.a) robot.intake.update(IntakeSubsystem.WristState.ACTIVE);
        //update * write


        telemetry.addData("Locked", locked);

    }

}