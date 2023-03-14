package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.AutoMidCycleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.common.vision.AprilTagDetector;

@Autonomous(name="Left Mid Cycle")
@Config
public class LeftMidCycleAuto extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap,true);


        robot.intake.update(IntakeSubsystem.ClawState.CLOSED);
        robot.intake.update(IntakeSubsystem.WristState.STOW);

        AutoMidCycleCommand.setTolerance(30);


        TrajectorySequence traj1 = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .forward(50)

                .build();



        while(!isStarted()){
            robot.vision.runAprilTag();
        }

        waitForStart();
        robot.intake.update(IntakeSubsystem.ClawState.OPEN);
        robot.drive.followTrajectorySequenceAsync(traj1);
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new WaitCommand(3000).andThen(new SequentialCommandGroup(
                                        //cycle
                                        new AutoMidCycleCommand(robot),
                                        new AutoMidCycleCommand(robot),
                                        new AutoMidCycleCommand(robot),
                                        new AutoMidCycleCommand(robot),
                                        new AutoMidCycleCommand(robot)
                                ))

                        )
                )
        );

        while (opModeIsActive()){
            robot.read();
            //parking code
            CommandScheduler.getInstance().run();

            robot.turret.loop();
            robot.vertical.loop();
            robot.horizontal.loop();

            robot.write();
        }
    }



}
