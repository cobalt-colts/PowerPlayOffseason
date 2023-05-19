package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SensorSubsystem extends SubsystemBase {
    private DigitalChannel receiver;
    private boolean state;
    public SensorSubsystem(HardwareMap hardwareMap){
        receiver = hardwareMap.get(DigitalChannel.class, "receiver");
        receiver.setMode(DigitalChannel.Mode.INPUT);
    }

    public void loop(){
        //idk
    }

    public void read() {
        //true: nothing is there, leave open
        //false: cone
        state = receiver.getState();
    }

    public boolean getState(){
        return state;
    }
}
