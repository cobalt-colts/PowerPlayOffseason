package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import static org.firstinspires.ftc.teamcode.common.drive.StandardTrackingWheelLocalizer.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.common.drive.StandardTrackingWheelLocalizer.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.common.drive.StandardTrackingWheelLocalizer.WHEEL_RADIUS;

import androidx.annotation.GuardedBy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.common.util.Encoder;

public class DriveSubsystem {
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private Encoder rightEncoder;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    private BNO055IMU imu;

    private double imuAngle = 0;
    private double zeroAngle = 0;

    private Thread imuThread;

    public DriveSubsystem(HardwareMap hardwareMap, boolean isAuto){
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightBack"));

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        synchronized (imuLock) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }
    }

    public void fieldRelative(double lsx,double lsy,double rsx,boolean calibrate){
        //@TODO Add Speed Modifs
        if (calibrate) {
            zeroAngle = this.getAngle();

        }
        double robotAngle = this.getAngle() - zeroAngle;

        double speed = Math.hypot(lsx, lsy); //get speed
        double LeftStickAngle = Math.atan2(lsy, -lsx) - Math.PI / 4; //get angle
        double rightX = rsx; //rotation
        rightX *= 0.8; //optionally reduce rotation value for better turning
        //linear the angle by the angle of the robot to make it field relative
        double leftFrontPower = speed * Math.sin(LeftStickAngle - robotAngle) + rightX; //+ when strafe (without reverse)
        double rightFrontPower = speed * Math.sin(LeftStickAngle - robotAngle) - rightX; //- when strafe
        double leftBackPower = speed * Math.cos(LeftStickAngle - robotAngle) + rightX; //- when strafe (without reverse)
        double rightBackPower = speed * Math.cos(LeftStickAngle - robotAngle) - rightX; //+ when strafe

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightBackPower);


    }

    public void startIMUThread(LinearOpMode opMode) {
        imuThread = new Thread(() -> {
            while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                synchronized (imuLock) {
                    imuAngle = -imu.getAngularOrientation().firstAngle;
                }
            }
        });
        imuThread.start();
    }

    public double getAngle() {
        return imuAngle;
    }

    public void setMotorPowers(double lf, double lb, double rf, double rb) {
        leftFront.setPower(lf);
        leftRear.setPower(lb);
        rightFront.setPower(rf);
        rightRear.setPower(rb);
    }

    public int getWheelPosition(){
        return (int) rightEncoder.getCurrentPosition();
    }

    public double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;

    }


}
