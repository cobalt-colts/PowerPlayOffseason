package org.firstinspires.ftc.teamcode.common.commandbase.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.VerticalSubsystem;

public class VerticalPositionCommand extends CommandBase {

    private VerticalSubsystem vertical;
    private int position;
    private double timeout;
    private double error;

    private ElapsedTime timer;

    public VerticalPositionCommand(VerticalSubsystem vertical, int position, double error, double timeout){
        this.vertical = vertical;
        this.position = position;
        this.timeout = timeout;
        this.error = error;
    }

    @Override
    public void initialize(){
        timer = new ElapsedTime();
        //@TODO FIX
        vertical.setTargetPos(position);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
