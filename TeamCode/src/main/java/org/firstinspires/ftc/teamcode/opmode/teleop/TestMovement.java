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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.common.util.Encoder;

@TeleOp
@Config
public class TestMovement extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private Encoder rightEncoder;
    private BNO055IMU imu;


    public double headingOffset = 0;
    public double robotHeading = 0;
    public double headingError  = 0;
    public double headingGoal = 0;
    public double encoderOffset = 0;


    
    public static PIDController fwd,rot;

    public static PIDCoefficients fwdVal = new PIDCoefficients(), rotVal = new PIDCoefficients();
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

        fwd = new PIDController(fwdVal.p, fwdVal.i, fwdVal.d);
        rot = new PIDController(rotVal.p, rotVal.i, rotVal.d);

        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            telemetry.update();
        }

        encoderOffset = encoderTicksToInches(getWheelPosition());
        resetHeading();

        while(opModeIsActive()){


        }


    }

    public void goDistance(int GOAL_DISTANCE){
        robotHeading = getRawHeading() - headingOffset;
        headingError = robotHeading - headingGoal;


        fwd.setPID(0.08, fwdVal.i, fwdVal.d);
        rot.setPID(0.08, rotVal.i, rotVal.d);

        double currPos = encoderTicksToInches(getWheelPosition()) - encoderOffset;
        telemetry.addData("curr: ",currPos);
        double fwdPower = Range.clip(fwd.calculate(currPos, GOAL_DISTANCE), -0.5, 0.5);

        double rotPower = Range.clip(rotVal.p * (robotHeading),-0.5,0.5);
        telemetry.addData("heading angle", robotHeading);
        telemetry.addData("heading err" , headingError);
        telemetry.addData("heading offset", headingOffset);
        telemetry.addData("heading pwr" , rotPower );
        leftFront.setPower(fwdPower + rotPower);
        leftRear.setPower(fwdPower + rotPower);

        rightFront.setPower(fwdPower - rotPower);
        rightRear.setPower(fwdPower - rotPower);

        telemetry.update();
    }

    public int getWheelPosition(){
        return (int) rightEncoder.getCurrentPosition();
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;

    }

    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
}
