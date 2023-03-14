package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class HorizontalSubsystem extends SubsystemBase {

    private DcMotorEx horSlide;
    private PIDController controller;

    public static double P = 0.002;
    public static double I = 0.0;
    public static double D = 0.00025;

    public static double maxVel = 2500;
    public static double maxAcc = 1900;

    private ElapsedTime voltageTimer;
    private VoltageSensor voltageSensor;
    private double voltage;

    private double power = 0.0;
    private int horizontalPosition;
    private int targetPosition = 0;

    //ty kookybotz
    public HorizontalSubsystem(HardwareMap hardwareMap, boolean isAuto){
        horSlide = hardwareMap.get(DcMotorEx.class, "horSlide");

        horSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new PIDController(P,I,D);
        controller.setPID(P, I, D);

        this.voltageTimer = new ElapsedTime();
        voltageTimer.reset();

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();
    }

    public void loop() {
        this.controller.setPID(P, I, D);

        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        power = (controller.calculate(horizontalPosition, targetPosition) / voltage * 14);
    }

    public void setTargetPos(double position) {
        targetPosition = (int) position;
    }

    public void setHorizontalFactor(double factor) {
        double horizontalAddition = 20 * factor;
        double newPosition = horizontalPosition + horizontalAddition;
        horizontalPosition = (int) newPosition;
    }

    public void read() {
        horizontalPosition = horSlide.getCurrentPosition();
    }

    public void write() {
        horSlide.setPower(power);
    }

    public void reset() { horSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); horSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER); }

    public int getPos() {
        return horizontalPosition;
    }

    public int getAbsError() { return Math.abs(getError());}

    public int getError() { return targetPosition - horizontalPosition;}
}
