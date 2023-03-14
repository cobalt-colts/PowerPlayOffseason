package org.firstinspires.ftc.teamcode.common.commandbase.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;

public class TurretPositionCommand extends CommandBase {

    private TurretSubsystem turret;
    private int position;
    private double timeout;
    private double error;

    private ElapsedTime timer;

    public TurretPositionCommand(TurretSubsystem horizontal, int position, double error, double timeout){
        this.turret = horizontal;
        this.position = position;
        this.timeout = timeout;
        this.error = error;
    }

    @Override
    public void initialize(){
        timer = new ElapsedTime();
        turret.setTargetPos(position);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
