package org.firstinspires.ftc.teamcode.opmode.auto.old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.RightAutoMidCycleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.HorizontalPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.TurretPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.VerticalPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@Autonomous(name="Right Mid Cycle")
@Config
public class RightMidCycleAuto extends LinearOpMode {
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
    public static PIDCoefficients fwdVal = new PIDCoefficients(0.08,0,0.02), rotVal = new PIDCoefficients(-3,0,0), strVal = new PIDCoefficients(0.6,0.005,0.02);
    public double voltage = 12;

    ElapsedTime et;
    @Override
    public void runOpMode() throws InterruptedException{
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap, telemetry,true);

        robot.intake.update(IntakeSubsystem.ClawState.CLOSED);
        robot.intake.update(IntakeSubsystem.WristState.STOW);

        RightAutoMidCycleCommand.setTolerance(30);

        fwd = new PIDController(fwdVal.p, fwdVal.i, fwdVal.d);
        rot = new PIDController(rotVal.p, rotVal.i, rotVal.d);
        str = new PIDController(strVal.p, strVal.i, strVal.d);

        et = new ElapsedTime();

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

            fwdEncoderOffset = robot.drive.getForwardPosition();
            strEncoderOffset = robot.drive.getLateralPosition();

            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

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
                        new RightAutoMidCycleCommand(robot),
                        new InstantCommand(() -> RightAutoMidCycleCommand.setStackHeight(350)),
                        new RightAutoMidCycleCommand(robot),
                        new InstantCommand(() -> RightAutoMidCycleCommand.setStackHeight(240)),
                        new RightAutoMidCycleCommand(robot),
                        new InstantCommand(() -> RightAutoMidCycleCommand.setStackHeight(210)),
                        new RightAutoMidCycleCommand(robot),
                        new InstantCommand(() -> RightAutoMidCycleCommand.setStackHeight(60)),
                        new RightAutoMidCycleCommand(robot),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> goPark()),
                                new VerticalPositionCommand(robot.vertical,0,30,5000),
                                new TurretPositionCommand(robot.turret,0,30,5000),
                                new HorizontalPositionCommand(robot.horizontal,0.1)

                        )
                )

        );


        while(et.seconds() < 6){
            moveRobot();

        }

        robot.drive.setMotorPowers(0,0,0,0);
        while (opModeIsActive()){
            robot.read();

            double currFwd = robot.drive.getForwardPosition() - fwdEncoderOffset;
            double currStr = robot.drive.getLateralPosition() - strEncoderOffset;

            CommandScheduler.getInstance().run();

            //@TODO verify
            robot.loop(true);

            robot.write();

            telemetry.addData("currFwd: ",currFwd);
            telemetry.addData("currStr: ",currStr);

            telemetry.addData("Vertical Pos:", robot.vertical.getPos());
            telemetry.addData("Vertical Target:", robot.vertical.getTargetPosition());
            telemetry.update();
        }
    }



    public void moveRobot(){
        robot.horizontal.horSlide.setPosition(0);

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

        double strafe = et.seconds() > 4? -6.5 : 0;

        double currFwd = robot.drive.getForwardPosition() - fwdEncoderOffset;
        double currStr = robot.drive.getLateralPosition() - strEncoderOffset;

        double fwdPower = Range.clip(fwd.calculate(currFwd, goal),-0.4,0.4) + 0.01 * Math.signum(goal-currFwd);
        double strPower = Range.clip(str.calculate(currStr,strafe), -0.5, 0.5) ;
        double rotPower = Range.clip(rot.calculate(headingError,0),-0.5,0.5);

        fwdPower *= voltage/12;
        strPower *= voltage/12;
        rotPower *= voltage/12;

//        telemetry.addData("uS position: ", robot.drive.getRawDistance());
//
//        telemetry.addData("heading angle", robotHeading);
//        telemetry.addData("heading err" , headingError);
//        telemetry.addData("heading offset", headingOffset);
//        telemetry.addData("heading pwr" , rotPower);


        telemetry.addData("----","----");

        telemetry.addData("currFwd: ",currFwd);
        telemetry.addData("currStr: ",currStr);
        telemetry.addData("fwd pwr: ",fwdPower);
        telemetry.addData("str pwr: ",strPower);

        robot.drive.leftFront.setPower(fwdPower + rotPower - strPower);
        robot.drive.leftRear.setPower(fwdPower + rotPower + strPower);
        robot.drive.rightFront.setPower(fwdPower - rotPower - strPower);
        robot.drive.rightRear.setPower(fwdPower - rotPower + strPower);

        robot.write();
        telemetry.update();
    }

    public void goPark(){

    }



}
