package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.left.LeftAutoMidCycleCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@Config
@Autonomous
public class TestCycleCommand extends LinearOpMode {
    Robot robot;

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
    public static int goal = 0;

    double loopTime;

    @Override
    public void runOpMode(){

        robot = new Robot(hardwareMap,telemetry,true);
        CommandScheduler.getInstance().reset();

        while(!isStarted()){
            fwdEncoderOffset = robot.drive.getForwardPosition();
            strEncoderOffset = robot.drive.getLateralPosition();

            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }


        waitForStart();
        while(opModeIsActive()){
            moveRobot(0,0);
            if(gamepad2.x){
                CommandScheduler.getInstance().reset();
                CommandScheduler.getInstance().schedule(new LeftAutoMidCycleCommand(robot,470));
            }
            robot.read(); //Gets position of multiple subsystems

            robot.loop(true); //runs PID

            telemetry.addData("Goal: ", goal);
            telemetry.addData("Current: ", robot.turret.getPos());
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000/(loop-loopTime));
            loopTime = loop;
            CommandScheduler.getInstance().run();


            telemetry.update();
            robot.write();


        }
    }

    public void moveRobot(double forward, double strafe){


        fwd = new ProfiledPIDController(fwdVal.p, fwdVal.i, fwdVal.d, new TrapezoidProfile.Constraints(max_vel,max_acc));
        rot = new PIDController(rotVal.p, rotVal.i, rotVal.d);
        str = new ProfiledPIDController(strVal.p, strVal.i, strVal.d, new TrapezoidProfile.Constraints(40,20));


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


        telemetry.addData("Vertical Pos:", robot.vertical.getPos());
        telemetry.addData("Vertical Target:", robot.vertical.getTargetPosition());

        telemetry.addData("----","----");

        telemetry.addData("currFwd: ",currFwd);
        telemetry.addData("currStr: ",currStr);
        telemetry.addData("fwd pwr: ",fwdPower);
        telemetry.addData("str pwr: ",strPower);

        robot.drive.leftFront.setPower(fwdPower + rotPower - strPower);
        robot.drive.leftRear.setPower(fwdPower + rotPower + strPower);
        robot.drive.rightFront.setPower(fwdPower - rotPower - strPower);
        robot.drive.rightRear.setPower(fwdPower - rotPower + strPower);

    }
}
