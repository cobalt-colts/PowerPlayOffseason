package org.firstinspires.ftc.teamcode.opmode.auto.right;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.right.RightAutoHighCycleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.right.RightAutoHighFinalCycleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.HorizontalPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.TurretPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.VerticalPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@Autonomous(name="Right High Motion Profiled Auton")
@Config
public class RightHighMotionProfiledAuto extends LinearOpMode {
    Robot robot;
    public int currId = 2;
    public boolean park = false;

    public enum Location {
        LEFT, MIDDLE, RIGHT
    }
    Location location = Location.MIDDLE;

    public static int goal = 49;

    public double headingOffset = 0;
    public double robotHeading = 0;
    public double headingError  = 0;
    public double headingGoal = 0;
    public double fwdEncoderOffset = 0;
    public double strEncoderOffset = 0;

    public double currFwd;
    public double currStr;

    public static ProfiledPIDController fwd,str;
    public static int max_vel = 50;
    public static int max_acc = 50;
    public static PIDController rot;
    public static PIDCoefficients fwdVal = new PIDCoefficients(0.07,0,0.02), rotVal = new PIDCoefficients(-3,0,0), strVal = new PIDCoefficients(0.3,0.005,0.02);
    public double voltage = 12;
    private double loopTime;
    private double endTime;

    ElapsedTime et;
    @Override
    public void runOpMode() throws InterruptedException{
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap, telemetry,true);


        robot.intake.update(IntakeSubsystem.ClawState.CLOSED);
        robot.intake.update(IntakeSubsystem.WristState.STOW);
        robot.intake.setGuidePosition(0);

        RightAutoHighCycleCommand.setTolerance(30);

        fwd = new ProfiledPIDController(fwdVal.p, fwdVal.i, fwdVal.d, new TrapezoidProfile.Constraints(max_vel,max_acc));
        rot = new PIDController(rotVal.p, rotVal.i, rotVal.d);
        str = new ProfiledPIDController(strVal.p, strVal.i, strVal.d, new TrapezoidProfile.Constraints(40,20));

        et = new ElapsedTime();

        while(!isStarted()){
            robot.vision.runAprilTag();
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

            robot.horizontal.horSlide.setPosition(0.9);

            fwdEncoderOffset = robot.drive.getForwardPosition();
            strEncoderOffset = robot.drive.getLateralPosition();

            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

            telemetry.addData("Location: ", currId);
            telemetry.update();
        }

        waitForStart();

        robot.vision.stopStreaming();

        headingOffset = robot.drive.imu.getAngularOrientation().firstAngle;

        robot.drive.startIMUThread(this);

        //robot.intake.update(IntakeSubsystem.WristState.ACTIVE);
        //robot.drive.followTrajectorySequenceAsync(traj1);
        et.reset();

        CommandScheduler.getInstance().schedule(


                //new DriveToCycleCommand(robot,telemetry,robot.drive.imu.getAngularOrientation().firstAngle),
                new SequentialCommandGroup(
                        //cycle
                        new RightAutoHighCycleCommand(robot,470),
                        new RightAutoHighCycleCommand(robot,360),
                        new RightAutoHighCycleCommand(robot,250),
                        new RightAutoHighFinalCycleCommand(robot,140),
                        new InstantCommand(() -> park = true)
                )

        );


        while(et.seconds() < 2){
            robot.read();

            robot.vertical.setTargetPos(1530);
            robot.turret.setTargetPos(-520);
            robot.horizontal.setPos(0.1);
            if(currFwd < 40) moveRobot(goal,0);
            else moveRobot(goal,-7);


            robot.loop(true);
            robot.write();
            telemetry.update();

        }

        robot.vertical.setTargetPos(1530);

        while (opModeIsActive()){

            robot.read();

            if(!park) {
                moveRobot(goal,-7);
            }else{
                robot.intake.update(IntakeSubsystem.WristState.STOW);
                moveRobot(goal+2,location == Location.RIGHT ? 0 : (location == Location.MIDDLE) ? 25 : 50);
            }
            CommandScheduler.getInstance().run();

            //@TODO verify
            robot.loop(true);

            robot.write();

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000/(loop-loopTime));
            loopTime = loop;

            telemetry.update();


        }
    }



    public void moveRobot(double forward, double strafe){


        fwd.setPID(fwdVal.p, fwdVal.i,fwdVal.d);
        rot.setPID(rotVal.p,rotVal.i,rotVal.d);
        str.setPID(strVal.p,strVal.i,strVal.d);

        robotHeading = robot.drive.getAngle() - headingOffset;
        headingError = robotHeading - headingGoal;

        while(headingError > Math.PI){
            headingError -= 2 * Math.PI;
        }

        while(headingError < -Math.PI){
            headingError += 2 * Math.PI;
        }



        currFwd = robot.drive.getForwardPosition() - fwdEncoderOffset;

        //double strafe = currFwd > 40? 6.5 : 0;

        currStr = robot.drive.getLateralPosition() - strEncoderOffset;

        double fwdPower = fwd.calculate(currFwd, forward) + 0.01 * Math.signum(forward-currFwd);
        double strPower = str.calculate(currStr,strafe);
        double rotPower = Range.clip(rot.calculate(headingError,0),-0.5,0.5);

        fwdPower *= voltage/14;
        strPower *= voltage/14;
        rotPower *= voltage/14;

        robot.drive.leftFront.setPower(fwdPower + rotPower - strPower);
        robot.drive.leftRear.setPower(fwdPower + rotPower + strPower);
        robot.drive.rightFront.setPower(fwdPower - rotPower - strPower);
        robot.drive.rightRear.setPower(fwdPower - rotPower + strPower);

        telemetry.addData("Curr Fwd: ", currFwd);
        telemetry.addData("Curr Str: ", currStr);
        telemetry.addData("Curr Vertical: ", robot.vertical.getPos());
        telemetry.addData("Curr Turret: ", robot.turret.getPos());
        //robot.write();
        //telemetry.update();
    }

}
