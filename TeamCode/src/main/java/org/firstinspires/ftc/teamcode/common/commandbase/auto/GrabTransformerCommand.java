package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.HorizontalPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.TurretPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.VerticalPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class GrabTransformerCommand extends SequentialCommandGroup {
    private static int TOLERANCE = 10;
    private static int stackHeight = 430;

    public GrabTransformerCommand(Robot robot){
        super(
                new SequentialCommandGroup(
                        //SCORE ON MEDIUM
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.WristState.ACTIVE)),
                        new WaitCommand(1000),
                        new HorizontalPositionCommand(robot.horizontal,0.36),
                        new WaitCommand(1000),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
                        new WaitCommand(1000),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.WristState.STOW)),
                        new VerticalPositionCommand(robot.vertical,0,30,5000),
                        new HorizontalPositionCommand(robot.horizontal,0)


                )
        );
    }

    public static void setTolerance(int tolerance){
        TOLERANCE = tolerance;
    }
    public static void setStackHeight(int s) { stackHeight = s;}
}
