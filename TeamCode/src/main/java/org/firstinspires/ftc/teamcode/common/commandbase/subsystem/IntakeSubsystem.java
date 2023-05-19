package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class IntakeSubsystem extends SubsystemBase {

    private Servo claw;
    private Servo wrist;

    private DigitalChannel receiver;

    public static double active = 0;
    public static double slight = 0.2;
    public static double stow = 0.8;

    public static double open = 0.4;
    public static double closed = 0;

    public enum ClawState{
        OPEN,
        CLOSED
    }

    public enum WristState{
        ACTIVE,
        SLIGHT,
        STOW
    }

    public IntakeSubsystem(HardwareMap hardwareMap, boolean isAuto){
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class,"wrist");


    }

    public void update(ClawState clawState){
        switch(clawState){
            case OPEN:
                claw.setPosition(open);
                break;
            case CLOSED:
                claw.setPosition(closed);
                break;
        }
    }

    public void update(WristState wristState){
        switch(wristState){
            case ACTIVE:
                wrist.setPosition(active);
                break;
            case SLIGHT:
                
            case STOW:
                wrist.setPosition(stow);
                break;
        }
    }

    //testing
    public void setClawPosition(double pos){ claw.setPosition(pos);}
    public void setWristPosition(double pos){ wrist.setPosition(pos);}

}
