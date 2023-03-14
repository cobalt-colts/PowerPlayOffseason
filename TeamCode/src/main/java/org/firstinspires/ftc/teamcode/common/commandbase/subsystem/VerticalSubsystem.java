package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class VerticalSubsystem extends SubsystemBase {

    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;
    private ProfiledPIDController controller;

    public static double P = 0.005;
    public static double I = 0.0;
    public static double D = 0.00001;
    public static double F = 0.001;

    public static double maxVel = 4000;
    public static double maxAcc = 2200;

    private ElapsedTime voltageTimer;
    private VoltageSensor voltageSensor;
    private double voltage;

    private double power = 0.0;
    private int verticalPosition;
    private int targetPosition = 0;

    private boolean isAuto = false;
    //ty kookybotz
    public VerticalSubsystem(HardwareMap hardwareMap, boolean isAuto){
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);


        controller = new ProfiledPIDController(P,I,D, new TrapezoidProfile.Constraints(maxVel,maxAcc));
        controller.setPID(P, I, D);

        this.voltageTimer = new ElapsedTime();
        voltageTimer.reset();

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();

        this.isAuto = isAuto;
    }

    public void loop() {
        this.controller.setPID(P, I, D);

        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        power = (controller.calculate(verticalPosition, targetPosition) / voltage * 14);
    }

    public void setTargetPos(double position) {
        targetPosition = (int) position;
    }

    public void setVerticalFactor(double factor) {
        double slideAddition = 20 * factor;
        double newPosition = verticalPosition + slideAddition;

        if(!isAuto && leftSlide.getCurrentPosition() < 0){
            //spool safety
            targetPosition = 50;
        }else{
            targetPosition = (int) newPosition;
        }
    }

    public void read() {
        verticalPosition = rightSlide.getCurrentPosition();
    }

    public void write() {
        rightSlide.setPower(power);
        leftSlide.setPower(1.2 * power);
    }

    public void reset() {
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public int getPos() {
        return verticalPosition;
    }

    public int getAbsError() { return Math.abs(getError());}

    public int getError() { return targetPosition - verticalPosition;}
}
