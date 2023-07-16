package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.left.LeftAutoMidCycleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.LeftCenterPreloadCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.HorizontalPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.TurretPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.VerticalPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmode.auto.right.RightHighMotionProfiledAuto;

@Autonomous(name="Left Center Preload")
@Config
public class LeftCenterPreloadAuto extends LinearOpMode {
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

    public double currFwd;
    public double currStr;

    public static ProfiledPIDController fwd,str;
    public static int max_vel = 30;
    public static int max_acc = 30;
    public static PIDController rot;
    public static PIDCoefficients fwdVal = new PIDCoefficients(0.07,0,0.02), rotVal = new PIDCoefficients(-3,0,0), strVal = new PIDCoefficients(0.3,0.005,0.02);
    public double voltage = 12;

    public boolean preload = false;
    public boolean transformer = false;
    public boolean goPark = false;

    ElapsedTime et;
    @Override
    public void runOpMode() throws InterruptedException{
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap, telemetry,true);

        robot.intake.update(IntakeSubsystem.ClawState.CLOSED);
        robot.intake.update(IntakeSubsystem.WristState.STOW);

        LeftAutoMidCycleCommand.setTolerance(30);

        fwd = new ProfiledPIDController(fwdVal.p, fwdVal.i, fwdVal.d, new TrapezoidProfile.Constraints(max_vel,max_acc));
        rot = new PIDController(rotVal.p, rotVal.i, rotVal.d);
        str = new ProfiledPIDController(strVal.p, strVal.i, strVal.d, new TrapezoidProfile.Constraints(30,20)); //40

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

            telemetry.addData("Location: ", currId);
            telemetry.update();

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

                        new LeftCenterPreloadCommand(robot),
                        new InstantCommand(() -> preload = true)
//                        new ParallelCommandGroup(
//                                new InstantCommand(() -> goPark()),
//                                new VerticalPositionCommand(robot.vertical,0,30,5000),
//                                new TurretPositionCommand(robot.turret,0,30,5000),
//                                new HorizontalPositionCommand(robot.horizontal,0.1)
//
//                        )
                )

        );


        while(et.seconds() < 5){
            if(currFwd < 48) moveRobot(52,0);
            else moveRobot(52,7);

            robot.loop(true);
            robot.write();
        }

        while (opModeIsActive()){
            robot.read();



            CommandScheduler.getInstance().run();


            if (preload){
                if(currFwd > 30){
                    moveRobot((currStr > 3)? 52 : 27,0);
                }else {
                    moveRobot(27, location == Location.RIGHT ? 0 : (location == Location.MIDDLE) ? 25 : -25);
                }
                //Center Auto parking is weird. L,M,R corresponds to 1,2,3 signal zones
            }else{
                moveRobot(52,7);
            }

            //@TODO verify
            robot.loop(true);

            robot.write();


            telemetry.addData("Vertical Pos:", robot.vertical.getPos());
            telemetry.addData("Vertical Target:", robot.vertical.getTargetPosition());
            telemetry.update();
        }
    }





    public void moveRobot(double forward, double strafe){

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
        telemetry.addData("Forward Power: ", fwdPower);
        fwdPower *= voltage/14;
        strPower *= voltage/14;
        rotPower *= voltage/14;

        robot.drive.leftFront.setPower(fwdPower + rotPower - strPower);
        robot.drive.leftRear.setPower(fwdPower + rotPower + strPower);
        robot.drive.rightFront.setPower(fwdPower - rotPower - strPower);
        robot.drive.rightRear.setPower(fwdPower - rotPower + strPower);

        telemetry.addData("Curr Fwd: ", currFwd);
        telemetry.addData("Curr Str: ", currStr);
        //robot.write();
        telemetry.update();
    }
    public void goPark(){

    }



}
