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
public class TurretSubsystem extends SubsystemBase {

    private DcMotorEx turret;
    private ProfiledPIDController controller;

    public static double P = 0.002;
    public static double I = 0.0;
    public static double D = 0.00025;

    public static double maxVel = 2500;
    public static double maxAcc = 1900;

    private ElapsedTime voltageTimer;
    private VoltageSensor voltageSensor;
    private double voltage;

    private double power = 0.0;
    private int turretPosition;
    private int targetPosition = 0;

    private boolean isAuto = false;
;

    //ty kookybotz
    public TurretSubsystem(HardwareMap hardwareMap, boolean isAuto){
        //@TODO HARDWARE MAP DUMBO
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new ProfiledPIDController(P,I,D, new TrapezoidProfile.Constraints(maxVel,maxAcc));
        controller.setPID(P, I, D);

        this.voltageTimer = new ElapsedTime();
        voltageTimer.reset();

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();

        this.isAuto = isAuto;
    }

    public void loop() {
        if(!isAuto) return;

        this.controller.setPID(P, I, D);

        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        power = (controller.calculate(turretPosition, targetPosition) / voltage * 14);
    }

    public void setTargetPos(double position) {
        targetPosition = (int) position;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void read() {
        turretPosition = turret.getCurrentPosition();
    }

    public void write() {
        turret.setPower(power);
    }

    public void reset() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getPower() { return power;}

    public int getPos() {
        return turretPosition;
    }

    public int getAbsError() { return Math.abs(getError());}

    public int getError() { return targetPosition - turretPosition;}
}
