package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.common.drive.StandardTrackingWheelLocalizer.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.common.drive.StandardTrackingWheelLocalizer.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.common.drive.StandardTrackingWheelLocalizer.WHEEL_RADIUS;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.common.util.Encoder;

@TeleOp
@Config
public class TestMovement extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private Encoder rightEncoder;
    private BNO055IMU imu;

    private double offset;

    public static int GOAL_DISTANCE = 50; //inches

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

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initIMU();

        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightBack"));

        fwd = new PIDController(fwdVal.p, fwdVal.i, fwdVal.d);
        rot = new PIDController(rotVal.p, rotVal.i, rotVal.d);

        while(opModeInInit()){
            offset = encoderTicksToInches(getWheelPosition());
            telemetry.addData("Offset: ", offset);
            telemetry.update();
        }

        while(opModeIsActive()){
            fwd.setPID(fwdVal.p, fwdVal.i, fwdVal.d);
            rot.setPID(rotVal.p, rotVal.i, rotVal.d);

            double currPos = encoderTicksToInches(getWheelPosition()) - offset;
            telemetry.addData("curr: ",currPos);
            double fwdPower = fwd.calculate(currPos, GOAL_DISTANCE);
            double rotPower = rotVal.p * (Math.toDegrees(this.getRawExternalHeading() + 157));
            telemetry.addData("angle", Math.toDegrees(this.getRawExternalHeading()) +157);
            leftFront.setPower(fwdPower + rotPower);
            leftRear.setPower(fwdPower + rotPower);

            rightFront.setPower(fwdPower - rotPower);
            rightRear.setPower(fwdPower - rotPower);

            telemetry.update();

        }


    }

    public int getWheelPosition(){
        return (int) rightEncoder.getCurrentPosition();
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;

    }

    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

    }

    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle  ;
    }
}
