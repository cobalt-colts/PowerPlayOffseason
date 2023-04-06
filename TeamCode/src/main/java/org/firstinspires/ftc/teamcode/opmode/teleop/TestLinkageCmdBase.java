package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.HorizontalLinkageSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

@Config
@TeleOp
public class TestLinkageCmdBase extends CommandOpMode {
    public static double linkagePos = 0.0;
    public static double clawPos = 0.0;
    public static double wristPos = 0.0;

    private HorizontalLinkageSubsystem horizontal;
    private IntakeSubsystem intake;

    private ElapsedTime timer;

//    GamepadEx driverOp = new GamepadEx(gamepad1);
//    GamepadEx toolOp = new GamepadEx(gamepad2);

    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();

        horizontal = new HorizontalLinkageSubsystem(hardwareMap, false);
        intake = new IntakeSubsystem(hardwareMap, false);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),this.telemetry);


    }


    @Override
    public void run(){
        if (timer == null) {
            timer = new ElapsedTime();
        }

        horizontal.setPos(linkagePos);
        intake.setClawPosition(clawPos);
        intake.setWristPosition(wristPos);
        horizontal.write();

    }




}
