package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.AutoMidCycleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.HorizontalPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.TurretPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.VerticalPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.common.vision.AprilTagDetector;

import java.util.HashMap;
import java.util.Map;

@Autonomous(name="Left Mid Cycle")
@Config
public class LeftMidCycleAuto extends LinearOpMode {
    Robot robot;
    public int currId = 2;
    public boolean park = false;

    public enum Location {
        LEFT, MIDDLE, RIGHT
    }
    Location location = Location.MIDDLE;
    TrajectorySequence traj1;
    TrajectorySequence left;
    TrajectorySequence right;

    @Override
    public void runOpMode() throws InterruptedException{
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap, telemetry,true);

        robot.intake.update(IntakeSubsystem.ClawState.CLOSED);
        robot.intake.update(IntakeSubsystem.WristState.STOW);

        AutoMidCycleCommand.setTolerance(30);


        traj1 = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .forward(50)

                .build();

        left = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .strafeLeft(25)

                .build();

        right = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .strafeRight(25)

                .build();


        while(!isStarted()){
            robot.vision.runAprilTag();
            telemetry.update();
            if(robot.vision.getAprilTag() != null) {
                currId = robot.vision.getAprilTag().id;
            }
            switch (currId){
                case 1:
                    location = Location.LEFT;
                    break;
                case 2:
                    location = Location.MIDDLE;
                    break;
                default:
                    location = Location.RIGHT;
                    break;
            }

        }

        waitForStart();
        robot.horizontal.setPos(0.1);
        robot.intake.update(IntakeSubsystem.WristState.ACTIVE);
        //robot.drive.followTrajectorySequenceAsync(traj1);
        CommandScheduler.getInstance().schedule(











                new SequentialCommandGroup(

                        new WaitCommand(1000).andThen(new SequentialCommandGroup(

                                //cycle
                                new AutoMidCycleCommand(robot),
                                new InstantCommand(() -> AutoMidCycleCommand.setStackHeight(280)),
                                new AutoMidCycleCommand(robot),
                                new InstantCommand(() -> AutoMidCycleCommand.setStackHeight(230)),
                                new AutoMidCycleCommand(robot),
                                new InstantCommand(() -> AutoMidCycleCommand.setStackHeight(160)),
                                new AutoMidCycleCommand(robot),
                                new InstantCommand(() -> AutoMidCycleCommand.setStackHeight(80)),
                                new AutoMidCycleCommand(robot),

                                new ParallelCommandGroup(
                                        new InstantCommand(() -> goPark()),
                                        new VerticalPositionCommand(robot.vertical,0,30,5000),
                                        new TurretPositionCommand(robot.turret,0,30,5000),
                                        new HorizontalPositionCommand(robot.horizontal,0.1)

                                )
                        ))
                )
        );

        while (opModeIsActive()){
            robot.read();

            CommandScheduler.getInstance().run();

            robot.loop();

            robot.write();

            telemetry.update();
        }
    }

    public void goPark(){
        if(currId == 1) robot.drive.followTrajectorySequenceAsync(left);
        if(currId == 3) robot.drive.followTrajectorySequenceAsync(right);
    }



}
