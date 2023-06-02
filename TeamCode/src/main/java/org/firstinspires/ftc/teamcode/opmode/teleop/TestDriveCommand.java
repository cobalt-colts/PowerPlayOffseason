package org.firstinspires.ftc.teamcode.opmode.teleop;

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


        waitForStart();

        robot.drive.startIMUThread(this);

        CommandScheduler.getInstance().schedule(
                new DriveToCycleCommand(robot)
        );

        while (opModeIsActive()){
            robot.read();

            CommandScheduler.getInstance().run();

            robot.loop();

            robot.write();

            telemetry.addData("Curr Heading: ", robot.drive.getAngle());
            telemetry.update();
        }
    }


}
