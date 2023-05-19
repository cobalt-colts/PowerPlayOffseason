package org.firstinspires.ftc.teamcode.common.hardware;

import androidx.annotation.GuardedBy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.HorizontalLinkageSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.SensorSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.VerticalSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.vision.AprilTagDetector;

public class Robot {

    public SampleMecanumDrive drive;
    public AprilTagDetector vision;
    public TurretSubsystem turret;
    public VerticalSubsystem vertical;
    public HorizontalLinkageSubsystem horizontal;
    public IntakeSubsystem intake;
    public SensorSubsystem sensor;

    private Telemetry telemetry;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    private BNO055IMU imu;
    private double imuAngle = 0;
    private double zeroAngle = 0;

    private Thread imuThread;

    private boolean isAuto = false;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean isAuto){
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        vision  = new AprilTagDetector();
        this.isAuto = isAuto;
        if(this.isAuto){
            vision.startAprilTag(hardwareMap);
        }else {
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
        horizontal = new HorizontalLinkageSubsystem(hardwareMap,isAuto);
        intake = new IntakeSubsystem(hardwareMap,isAuto);
        sensor = new SensorSubsystem(hardwareMap);
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

        drive.leftFront.setPower(leftFrontPower);
        drive.leftRear.setPower(leftBackPower);
        drive.rightFront.setPower(rightFrontPower);
        drive.rightRear.setPower(rightBackPower);


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
        sensor.read();
        telemetry.addData("Receiver Open: ", sensor.getState());
        telemetry.addData("Turret Pos: ", turret.getPos());
        telemetry.addData("Turret Goal: ", turret.getTargetPosition());

        telemetry.addData("Vertical Pos: ", vertical.getPos());
        telemetry.addData("Vertical Goal: ", vertical.getTargetPosition());

    }

    public void loop(){
        if(isAuto) drive.update();
        turret.loop();
        if(isAuto) vertical.loop();
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

    }
}
