package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@TeleOp
public class Opmode extends CommandOpMode {
    private Robot robot;
    private ElapsedTime timer;

    GamepadEx driverOp;
    GamepadEx toolOp;

    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();

        robot = new Robot(hardwareMap,false);
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

        robot.read();

        //drivetrain
        robot.fieldRelative(driverOp.getLeftX(), driverOp.getLeftY(), driverOp.getRightX(), driverOp.getButton(GamepadKeys.Button.X));

        //turret
        robot.turret.setPower(0.5 * toolOp.getRightX());

        //vertical
        robot.vertical.setPower(-toolOp.getLeftY());

        //horizontal
        //@TODO fix (tired lol)

        //claw
        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).or(
                toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
        ).whenActive(new InstantCommand(() -> {
            robot.intake.update(IntakeSubsystem.ClawState.OPEN);
        }));

        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).or(
                toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
        ).whenActive(new InstantCommand(() -> {
            robot.intake.update(IntakeSubsystem.ClawState.CLOSED);
        }));

        //wrist
        toolOp.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(() -> {
                    robot.intake.update(IntakeSubsystem.WristState.STOW);
                })
        );

        toolOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> {
                    robot.intake.update(IntakeSubsystem.WristState.ACTIVE);
                })
        );

        //update * write
        robot.loop();
        robot.write();
    }




}
