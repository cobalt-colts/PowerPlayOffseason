package org.firstinspires.ftc.teamcode.common.hardware;

import static java.lang.Thread.sleep;

import androidx.annotation.GuardedBy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.HorizontalLinkageSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.SensorSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.VerticalSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.vision.AprilTagDetector;

import java.util.List;

public class Robot {

    List<LynxModule> allHubs;

    public AprilTagDetector vision;
    public TurretSubsystem turret;
    public VerticalSubsystem vertical;
    public HorizontalLinkageSubsystem horizontal;
    public IntakeSubsystem intake;
    public SensorSubsystem sensor;
    public DriveSubsystem drive;

    private Telemetry telemetry;


    private boolean isAuto = false;

    private ElapsedTime rumble;
    private boolean prevTouchpad = false;
    private boolean currTouchpad = false;

    private boolean autoPickup = false;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean isAuto){

        allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        vision  = new AprilTagDetector();
        this.isAuto = isAuto;
        if(this.isAuto){
            vision.startAprilTag(hardwareMap);
        }
        //else (commented out if not using RR)


        drive = new DriveSubsystem(hardwareMap,isAuto);
        turret = new TurretSubsystem(hardwareMap,isAuto);
        vertical = new VerticalSubsystem(hardwareMap,isAuto);
        horizontal = new HorizontalLinkageSubsystem(hardwareMap,isAuto);
        intake = new IntakeSubsystem(hardwareMap,isAuto);
        sensor = new SensorSubsystem(hardwareMap);

        rumble = new ElapsedTime();
    }


    public void updateClaw(Gamepad gamepad1, Gamepad gamepad2) {
        boolean currState = sensor.getCurrState();
        boolean prevState = sensor.getPrevState();
        //true: nothing is there, leave open
        //false: cone
        if(rumble.milliseconds() > 300){
            gamepad1.stopRumble();
            gamepad2.stopRumble();
            rumble.reset();
        }

        currTouchpad = gamepad1.touchpad;
        if(currTouchpad && !prevTouchpad) {
            autoPickup = !autoPickup;
        }

        if(!currState && prevState){
            gamepad1.rumble(0.5, 0.5, Gamepad.RUMBLE_DURATION_CONTINUOUS);
            gamepad2.rumble(0.5, 0.5, Gamepad.RUMBLE_DURATION_CONTINUOUS);
            rumble.startTime();
        }

        if(autoPickup && !(gamepad1.left_bumper || gamepad2.left_bumper)){
            if(currState) intake.update(IntakeSubsystem.ClawState.OPEN);
            else intake.update(IntakeSubsystem.ClawState.CLOSED);
        }else if(autoPickup && (gamepad1.left_bumper || gamepad2.left_bumper)){
            intake.update(IntakeSubsystem.ClawState.OPEN);
            try {
                Thread.sleep(600);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }else{
            if(gamepad1.left_bumper || gamepad2.left_bumper) intake.update(IntakeSubsystem.ClawState.OPEN);
            if(gamepad1.right_bumper || gamepad2.right_bumper) intake.update(IntakeSubsystem.ClawState.CLOSED);
        }
    }



    public void read(){
        turret.read();
        vertical.read();
        sensor.read();
//        telemetry.addData("Receiver Open: ", sensor.getCurrState());
//        telemetry.addData("Turret Pos: ", turret.getPos());
//        telemetry.addData("Turret Goal: ", turret.getTargetPosition());
//
//        telemetry.addData("Vertical Pos: ", vertical.getPos());
//        telemetry.addData("Vertical Goal: ", vertical.getTargetPosition());

    }

    public void loop(boolean auto, boolean overrideTurret){
        //if(isAuto) drive.update();

        //if auto is true, then we enable PID control
        turret.loop(overrideTurret);
        vertical.loop(auto);
        horizontal.loop();

    }

    public void loop(boolean auto){
        //if(isAuto) drive.update();

        //if auto is true, then we enable PID control
        turret.loop(auto);
        vertical.loop(auto);
        horizontal.loop();

    }

    public void write(){
        turret.write();
        vertical.write();
        horizontal.write();
        sensor.write();

        for(LynxModule hub: allHubs){
            hub.clearBulkCache();
        }


    }

    public void reset(){
        turret.reset();
        vertical.reset();

    }
}
