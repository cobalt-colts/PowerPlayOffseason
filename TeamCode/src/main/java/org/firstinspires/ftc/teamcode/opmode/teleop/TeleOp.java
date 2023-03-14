package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class TeleOp extends CommandOpMode {
    private Robot robot;
    private ElapsedTime timer;

    GamepadEx driverOp = new GamepadEx(gamepad1);
    GamepadEx toolOp = new GamepadEx(gamepad2);

    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();

        robot = new Robot(hardwareMap,false);
        robot.reset();
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),this.telemetry);


    }


    @Override
    public void run(){
        if (timer == null) {
            timer = new ElapsedTime();
            robot.reset();
            robot.startIMUThread(this);
        }

        robot.read();

        robot.fieldRelative(driverOp.getLeftX(), driverOp.getLeftY(), driverOp.getRightX(), driverOp.getButton(GamepadKeys.Button.X));

        robot.turret.setTurretFactor(toolOp.getRightX());
        robot.vertical.setVerticalFactor(-toolOp.getLeftY());
        robot.horizontal.setHorizontalFactor(toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) ?
                -toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) : toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) );

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

        robot.loop();
        robot.write();
    }




}
