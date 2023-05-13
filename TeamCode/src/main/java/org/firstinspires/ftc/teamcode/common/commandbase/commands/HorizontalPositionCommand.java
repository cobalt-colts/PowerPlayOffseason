package org.firstinspires.ftc.teamcode.common.commandbase.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.HorizontalLinkageSubsystem;

public class HorizontalPositionCommand extends CommandBase {

    private HorizontalLinkageSubsystem horizontal;
    private double position;

    public HorizontalPositionCommand(HorizontalLinkageSubsystem horizontal, double position){
        this.horizontal = horizontal;
        this.position = position;
    }

    @Override
    public void initialize(){
        horizontal.setPos(position);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
