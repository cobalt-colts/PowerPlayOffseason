package org.firstinspires.ftc.teamcode.common.commandbase.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.HorizontalSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.VerticalSubsystem;

public class HorizontalPositionCommand extends CommandBase {

    private HorizontalSubsystem horizontal;
    private int position;
    private double timeout;
    private double error;

    private ElapsedTime timer;

    public HorizontalPositionCommand(HorizontalSubsystem horizontal, int position, double error, double timeout){
        this.horizontal = horizontal;
        this.position = position;
        this.timeout = timeout;
        this.error = error;
    }

    @Override
    public void initialize(){
        timer = new ElapsedTime();
        horizontal.setTargetPos(position);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
