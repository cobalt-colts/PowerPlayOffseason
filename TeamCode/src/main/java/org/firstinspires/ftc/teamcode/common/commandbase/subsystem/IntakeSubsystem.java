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
    private Servo guide;

    private DigitalChannel receiver;

    public static double active = 0;
    public static double slight = 0.2;
    public static double stow = 0.8;

    public static double open = 1;
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

    public enum GuideState{
        LEFT,
        MIDLEFT,
        MIDRIGHT,
        RIGHT
    }

    public IntakeSubsystem(HardwareMap hardwareMap, boolean isAuto){
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class,"wrist");
        guide = hardwareMap.get(Servo.class, "guide");


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
                wrist.setPosition(slight);
                break;
            case STOW:
                wrist.setPosition(stow);
                break;
        }
    }

    public void update(GuideState guideState){
        switch(guideState){
            case LEFT:
                wrist.setPosition(0);
                break;
            case MIDLEFT:
                wrist.setPosition(0.3);
                break;
            case MIDRIGHT:
                wrist.setPosition(0.7);
                break;
            case RIGHT:
                wrist.setPosition(1);
                break;
        }
    }
    //testing
    public void setClawPosition(double pos){ claw.setPosition(pos);}
    public void setWristPosition(double pos){ wrist.setPosition(pos);}
    public void setGuidePosition(double pos){ guide.setPosition(pos);}

}
