package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.common.drive.StandardTrackingWheelLocalizer.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.common.drive.StandardTrackingWheelLocalizer.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.common.drive.StandardTrackingWheelLocalizer.WHEEL_RADIUS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.util.Encoder;

@TeleOp
@Config
public class TestMovement extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private Encoder rightEncoder, frontEncoder;
    private BNO055IMU imu;

    public static int goal = 0;
    public static int strafe = 0;

    public double headingOffset = 0;
    public double robotHeading = 0;
    public double headingError  = 0;
    public double headingGoal = 0;
    public double fwdEncoderOffset = 0;
    public double strEncoderOffset = 0;

    
    public static PIDController fwd,rot,str;

    public static PIDCoefficients fwdVal = new PIDCoefficients(0.2,0,0), rotVal = new PIDCoefficients(4,0,0), strVal = new PIDCoefficients(0.5,0,0);
    @Override
    public void runOpMode(){
        leftFront = hardwareMap.get(DcMotorEx .class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initIMU();

        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightBack"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));

        fwd = new PIDController(0.8, fwdVal.i, fwdVal.d);
        rot = new PIDController(4, rotVal.i, rotVal.d);
        str = new PIDController(strVal.p, strVal.i, strVal.d);

        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            telemetry.addData("Forward Encoder Value: ", encoderTicksToInches(getFwd()));
            telemetry.update();
        }

        fwdEncoderOffset = getFwd();
        strEncoderOffset = getStr();

        resetHeading();

        while(opModeIsActive()){
            robotHeading = getRawHeading() - headingOffset;
            headingError = robotHeading - headingGoal;


            fwd.setPID(fwdVal.p, fwdVal.i,fwdVal.d);
            rot.setPID(rotVal.p,rotVal.i,rotVal.d);
            str.setPID(strVal.p,strVal.i,strVal.d);
            double currFwd = getFwd() - fwdEncoderOffset;
            double currStr = getStr() - strEncoderOffset;

            double fwdPower = Range.clip(fwd.calculate(currFwd, goal),-0.3,0.3) + 0.02 * Math.signum(goal-currFwd);
            double strPower = Range.clip(str.calculate(currStr,strafe), -0.5, 0.5);
            double rotPower = Range.clip(rotVal.p * (robotHeading),-0.5,0.5);

            telemetry.addData("heading angle", robotHeading);
            telemetry.addData("heading err" , headingError);
            telemetry.addData("heading offset", headingOffset);
            telemetry.addData("heading pwr" , rotPower);

            telemetry.addData("----","----");

            telemetry.addData("currFwd: ",currFwd);
            telemetry.addData("currStr: ",currStr);
            telemetry.addData("fwd pwr: ",fwdPower);
            telemetry.addData("str pwr: ",strPower);

            leftFront.setPower(fwdPower + rotPower - strPower);
            leftRear.setPower(fwdPower + rotPower + strPower);
            rightFront.setPower(fwdPower - rotPower - strPower);
            rightRear.setPower(fwdPower - rotPower + strPower);

            telemetry.update();

        }


    }

    public double getFwd(){
        return encoderTicksToInches(rightEncoder.getCurrentPosition());
    }

    public double getStr(){
        return encoderTicksToInches(frontEncoder.getCurrentPosition());
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;

    }

    public void initIMU() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

    }

    public double getRawHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
}
