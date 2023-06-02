package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SensorSubsystem extends SubsystemBase {
    private DigitalChannel receiver;
    private boolean prevState = true;
    private boolean currState;
    public SensorSubsystem(HardwareMap hardwareMap){
        receiver = hardwareMap.get(DigitalChannel.class, "receiver");
        receiver.setMode(DigitalChannel.Mode.INPUT);
    }

    public void loop(){
        //idk
    }

    public void read() {
        currState = receiver.getState();
    }


    public void write(){
        prevState = currState;
    }
    public boolean getCurrState(){
        return currState;
    }

    public boolean getPrevState() { return prevState; }
}
