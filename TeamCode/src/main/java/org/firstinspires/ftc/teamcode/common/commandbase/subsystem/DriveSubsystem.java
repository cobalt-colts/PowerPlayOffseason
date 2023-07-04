package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import static org.firstinspires.ftc.teamcode.common.drive.StandardTrackingWheelLocalizer.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.common.drive.StandardTrackingWheelLocalizer.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.common.drive.StandardTrackingWheelLocalizer.WHEEL_RADIUS;

import androidx.annotation.GuardedBy;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.util.Encoder;

@Config
public class DriveSubsystem {
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private Encoder rightEncoder,frontEncoder;
    public AnalogInput distanceSensor;

    public static double kP = 0;
    public static PIDCoefficients rotVal = new PIDCoefficients(4,0,0);
    
    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public BNO055IMU imu;

    private double imuAngle = 0;
    private double zeroAngle = 0;
    private double lastVoltage = 0;

    public double lockedTarget;
    public boolean init = true;
    private Thread imuThread;

    public DriveSubsystem(HardwareMap hardwareMap, boolean isAuto){
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightBack"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));

        distanceSensor = hardwareMap.get(AnalogInput.class, "distanceSensor");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        synchronized (imuLock) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }
    }
    
    public void lockedFieldRelative(double lsx, double lsy, double rsx){
        if(rsx != 0) {
            lockedTarget = getAngle();
        }

        double error = getAngle() - lockedTarget;
        if(init) error = 0;
        while(error > Math.PI){
            error -= 2 * Math.PI;
        }

        while(error < -Math.PI){
            error += 2 * Math.PI;
        }
        
        double y = lsy;
        double x = -lsx * 1.1;
        double rx = rsx;

        double denom = Math.max(Math.abs(y) + Math.abs(x),1);


        setMotorPowers((y+x+rx)/denom + error * kP, (y-x+rx)/denom + error * kP, (y+x-rx)/denom - error * kP,(y-x-rx)/denom - error * kP);

    }
    public void fieldRelative(double lsx,double lsy,double rsx,boolean calibrate){
        //@TODO Add Speed Modifs
        if (calibrate) {
            zeroAngle = this.getNegativeAngle();

        }
        double robotAngle = this.getNegativeAngle() - zeroAngle;

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
                    //@TODO may need to switch signs
                    imuAngle = imu.getAngularOrientation().firstAngle;
                }
            }
        });
        imuThread.start();
    }

    public double getAngle() {
        return imuAngle;
    }

    public double getNegativeAngle() {return -imuAngle;}
    public void setMotorPowers(double lf, double lb, double rf, double rb) {
        leftFront.setPower(lf);
        leftRear.setPower(lb);
        rightFront.setPower(rf);
        rightRear.setPower(rb);
    }

    public double getForwardPosition(){
        return  encoderTicksToInches(rightEncoder.getCurrentPosition());
    }

    public double getLateralPosition() {
        return  encoderTicksToInches(frontEncoder.getCurrentPosition());
    }

    public double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;

    }

    public double getRawDistance() {
        double rawVoltage = distanceSensor.getVoltage();
        double reading = rawVoltage*  542.1822921180930552;
        if (rawVoltage > 1.4823) { //1.4823 volts is about 144 inches, the width of the field.  If the volate is greater then this, there must have been a faulty reading
            rawVoltage = lastVoltage; //use last distance instead of new one.
        } else if(reading < 70 || reading > 82){
            rawVoltage = lastVoltage;
            reading = rawVoltage * 542.1822921180930552;
        }else{
            lastVoltage = rawVoltage;
        }

        //Constants gotten from getting the true distance from distance sensor to wall via tape measure,
        //then (distance/voltage) got the values in inches, then converted that to other distance units

        return reading;

    }

}
