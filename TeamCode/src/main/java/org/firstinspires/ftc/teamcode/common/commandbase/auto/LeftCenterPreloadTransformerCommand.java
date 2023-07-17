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

public class LeftCenterPreloadTransformerCommand extends SequentialCommandGroup {
    private static int TOLERANCE = 10;
    private static int stackHeight = 430;

    public LeftCenterPreloadTransformerCommand(Robot robot){
        super(
                new SequentialCommandGroup(
                        //SCORE ON MEDIUM
                        new VerticalPositionCommand(robot.vertical, 1520,0,5000),

                        new WaitUntilCommand(() -> robot.vertical.getPos() > 600),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.WristState.SLIGHT)),
                        new HorizontalPositionCommand(robot.horizontal,0.0),
                        new TurretPositionCommand(robot.turret,-400,0,5000),

                        //.andThen(new WaitCommand(500)),
                        new WaitUntilCommand(() -> robot.vertical.controller.atGoal() && robot.turret.controller.atGoal())
                                .andThen(new HorizontalPositionCommand(robot.horizontal, 0.32))
                                .andThen(new WaitCommand(500)),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.WristState.ACTIVE))
                                .andThen(new InstantCommand((() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN))))
                                .andThen(new WaitCommand(500)),


                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.WristState.STOW)),
                                new HorizontalPositionCommand(robot.horizontal,0),
                                new WaitCommand(300),
                                new VerticalPositionCommand(robot.vertical,100,30,5000),
                                new TurretPositionCommand(robot.turret,-800,30,5000)


                )
        );
    }

    public static void setTolerance(int tolerance){
        TOLERANCE = tolerance;
    }
    public static void setStackHeight(int s) { stackHeight = s;}
}
