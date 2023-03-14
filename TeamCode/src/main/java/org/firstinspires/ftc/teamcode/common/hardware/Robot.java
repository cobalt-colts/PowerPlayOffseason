package org.firstinspires.ftc.teamcode.common.hardware;

import androidx.annotation.GuardedBy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.HorizontalSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.VerticalSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.vision.AprilTagDetector;

public class Robot {

    public SampleMecanumDrive drive;
    public AprilTagDetector vision;
    public TurretSubsystem turret;
    public VerticalSubsystem vertical;
    public HorizontalSubsystem horizontal;
    public IntakeSubsystem intake;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    private BNO055IMU imu;
    private double imuAngle = 0;
    private double zeroAngle = 0;

    private Thread imuThread;

    private boolean isAuto = false;

    public Robot(HardwareMap hardwareMap, boolean isAuto){
        this.isAuto = isAuto;
        vision.startAprilTag();

        if(!isAuto) {
            synchronized (imuLock) {
                imu = hardwareMap.get(BNO055IMU.class, "imu");
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(parameters);
            }
        }

        drive = new SampleMecanumDrive(hardwareMap);
        turret = new TurretSubsystem(hardwareMap,isAuto);
        vertical = new VerticalSubsystem(hardwareMap,isAuto);
        horizontal = new HorizontalSubsystem(hardwareMap,isAuto);
        intake = new IntakeSubsystem(hardwareMap,isAuto);
    }

    public void fieldRelative(double lsx,double lsy,double rsx,boolean calibrate){
        //@TODO Add Speed Modifs

        if (calibrate) {
            zeroAngle = this.getAngle();

        }
        double robotAngle = this.getAngle() - zeroAngle;

        double speed = Math.hypot(lsx, lsy); //get speed
        double LeftStickAngle = Math.atan2(lsy, -lsx) - Math.PI / 4; //get angle
        double rightX = -rsx; //rotation
        rightX *= 0.8; //optionally reduce rotation value for better turning
        //linear the angle by the angle of the robot to make it field relative
        double leftFrontPower = speed * Math.cos(LeftStickAngle - robotAngle) + rightX;
        double rightFrontPower = speed * Math.sin(LeftStickAngle - robotAngle) - rightX;
        double leftBackPower = speed * Math.sin(LeftStickAngle - robotAngle) + rightX;
        double rightBackPower = speed * Math.cos(LeftStickAngle - robotAngle) - rightX;

        drive.setMotorPowers(leftFrontPower,leftBackPower,rightFrontPower,rightBackPower);

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

    public void read(){
        turret.read();
        vertical.read();
        horizontal.read();
    }

    public void loop(){
        turret.loop();
        vertical.loop();
        horizontal.loop();
    }

    public void write(){
        turret.write();
        vertical.write();
        horizontal.write();
    }

    public void reset(){
        turret.reset();
        vertical.reset();
        horizontal.reset();
    }
}
