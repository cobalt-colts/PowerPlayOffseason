package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.DriveToCycleCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@Autonomous
public class TestDriveCommand extends LinearOpMode {
    private Robot robot;
    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap,telemetry,true);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);

        while(opModeInInit()){
            telemetry.addData("Curr Heading: ", robot.drive.imu.getAngularOrientation().firstAngle);
            telemetry.update();
        }
        waitForStart();

        robot.drive.startIMUThread(this);
        DriveToCycleCommand d = new DriveToCycleCommand(robot, this.telemetry, robot.drive.imu.getAngularOrientation().firstAngle,robot.drive.getForwardPosition(),robot.drive.getLateralPosition());
        CommandScheduler.getInstance().schedule(
                d
        );

        while (opModeIsActive()){
            robot.read();
            robot.turret.setTargetPos(0);
            CommandScheduler.getInstance().run();

            robot.loop();

            robot.write();

            telemetry.addData("Forward Offset",d.getFwdWheelOffset());
            telemetry.addData("Lateral Offset",d.getLatWheelOffset());
            telemetry.addData("Heading Offset",d.getHeadingOffset());
            telemetry.update();
        }
    }


}
