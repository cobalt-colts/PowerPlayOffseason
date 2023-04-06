package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class HorizontalLinkageSubsystem extends SubsystemBase {

    private Servo horSlide;

    private double targetPosition;
    private double previousPosition = 0.0;

    //ty kookybotz
    public HorizontalLinkageSubsystem(HardwareMap hardwareMap, boolean isAuto){
        horSlide = hardwareMap.get(Servo.class, "horLinkage");

    }

    public void loop() {

    }

    public double getPos() {
        return targetPosition;
    }
    public double getPrevPos() { return previousPosition; }

    public void setPos(double position) {
        targetPosition = position;
    }


    public void write() {
        horSlide.setPosition(targetPosition);
        previousPosition = targetPosition;
    }



}
