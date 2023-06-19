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
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.AutoMidCycleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.DriveToCycleCommand;
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

    public static int goal = 50;

    public double headingOffset = 0;
    public double robotHeading = 0;
    public double headingError  = 0;
    public double headingGoal = 0;
    public double fwdEncoderOffset = 0;
    public double strEncoderOffset = 0;

    public static PIDController fwd,rot,str;

    ElapsedTime et;
    @Override
    public void runOpMode() throws InterruptedException{
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap, telemetry,true);

        robot.intake.update(IntakeSubsystem.ClawState.CLOSED);
        robot.intake.update(IntakeSubsystem.WristState.STOW);

        AutoMidCycleCommand.setTolerance(30);

        fwd = new PIDController(0.2, 0, 0);
        rot = new PIDController(4, 0, 0);
        str = new PIDController(1,0,0.02);

        et = new ElapsedTime();

        while(!isStarted()){
            robot.vision.runAprilTag();
            telemetry.addData("uS position: ", robot.drive.getRawDistance());
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

            fwdEncoderOffset = robot.drive.getForwardPosition();
            strEncoderOffset = robot.drive.getLateralPosition();


        }

        waitForStart();

        headingOffset = robot.drive.imu.getAngularOrientation().firstAngle;

        robot.drive.startIMUThread(this);

        robot.horizontal.setPos(0.1);
        robot.intake.update(IntakeSubsystem.WristState.ACTIVE);
        //robot.drive.followTrajectorySequenceAsync(traj1);
        et.reset();

        CommandScheduler.getInstance().schedule(


                        //new DriveToCycleCommand(robot,telemetry,robot.drive.imu.getAngularOrientation().firstAngle),
                        new SequentialCommandGroup(

                                //cycle
                                new WaitCommand(3000),
                                new AutoMidCycleCommand(robot),
                                new InstantCommand(() -> AutoMidCycleCommand.setStackHeight(260)),
                                new AutoMidCycleCommand(robot),
                                new InstantCommand(() -> AutoMidCycleCommand.setStackHeight(210)),
                                new AutoMidCycleCommand(robot),
                                new InstantCommand(() -> AutoMidCycleCommand.setStackHeight(140)),
                                new AutoMidCycleCommand(robot),
                                new InstantCommand(() -> AutoMidCycleCommand.setStackHeight(60)),
                                new AutoMidCycleCommand(robot),

                                new ParallelCommandGroup(
                                        new InstantCommand(() -> goPark()),
                                        new VerticalPositionCommand(robot.vertical,0,30,5000),
                                        new TurretPositionCommand(robot.turret,0,30,5000),
                                        new HorizontalPositionCommand(robot.horizontal,0.1)

                                )
                        )

        );


        while(et.seconds() < 9){
            robotHeading = robot.drive.getAngle() - headingOffset;
            headingError = robotHeading - headingGoal;

            double strafe = et.seconds() > 6? 5 : 0;

            double currFwd = robot.drive.getForwardPosition() - fwdEncoderOffset;
            double currStr = robot.drive.getLateralPosition() - strEncoderOffset;

            double fwdPower = Range.clip(fwd.calculate(currFwd, goal),-0.3,0.3) + 0.02 * Math.signum(goal-currFwd);
            double strPower = Range.clip(str.calculate(currStr,strafe), -0.5, 0.5) ;
            double rotPower = Range.clip(5 * (robotHeading),-0.5,0.5);



            telemetry.addData("uS position: ", robot.drive.getRawDistance());

            telemetry.addData("----","----");

            telemetry.addData("currFwd: ",currFwd);
            telemetry.addData("currStr: ",currStr);
            telemetry.addData("fwd pwr: ",fwdPower);
            telemetry.addData("str pwr: ",strPower);

            robot.drive.leftFront.setPower(fwdPower + rotPower - strPower);
            robot.drive.leftRear.setPower(fwdPower + rotPower + strPower);
            robot.drive.rightFront.setPower(fwdPower - rotPower - strPower);
            robot.drive.rightRear.setPower(fwdPower - rotPower + strPower);

            telemetry.update();

        }

        robot.drive.setMotorPowers(0,0,0,0);
        while (opModeIsActive()){
            robot.read();

            CommandScheduler.getInstance().run();

            robot.loop();

            robot.write();

            telemetry.update();
        }
    }

    public void goPark(){

    }



}
