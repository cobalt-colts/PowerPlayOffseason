package org.firstinspires.ftc.teamcode.common.commandbase.auto.left;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.HorizontalPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.TurretPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.VerticalPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class LeftAutoHighCycleCommand extends SequentialCommandGroup {
    private static int TOLERANCE = 10;

    public LeftAutoHighCycleCommand(Robot robot, int stackHeight){
        super(
                new SequentialCommandGroup(
                        //SCORE ON HIGH
                        new VerticalPositionCommand(robot.vertical, 1520,0,5000),

                        new WaitUntilCommand(() -> robot.vertical.getPos() > stackHeight + 100),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.WristState.SLIGHT)),
                        new HorizontalPositionCommand(robot.horizontal,0.05),
                        new TurretPositionCommand(robot.turret,430,0,5000),
                        new WaitUntilCommand(() -> robot.turret.getPos() > -300), //@TODO fix

                        //.andThen(new WaitCommand(500)),
                        new WaitUntilCommand(() -> robot.vertical.controller.atGoal() && robot.turret.controller.atGoal())
                                .andThen(new HorizontalPositionCommand(robot.horizontal, 0.32))
                                .andThen(new WaitCommand(200)),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.WristState.ACTIVE))
                                .andThen(new InstantCommand((() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN))))
                                .andThen(new WaitCommand(200))
                                .andThen(new HorizontalPositionCommand(robot.horizontal, 0.1))
                                .andThen(new WaitCommand(200)),

                        //INTAKE FROM STACK
                        new TurretPositionCommand(robot.turret,-820,0,5000),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.WristState.ACTIVE)),
                        new WaitUntilCommand(() -> robot.turret.getPos() > (-2160 + 200)),

                        new VerticalPositionCommand(robot.vertical, stackHeight,0,5000),
                        new WaitUntilCommand(() -> robot.vertical.controller.atGoal() && robot.turret.controller.atGoal()),
                        //swapped 42/43 with 45
                        new HorizontalPositionCommand(robot.horizontal,0.4).andThen(new WaitCommand(600)),


                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)).andThen(new WaitCommand(200))
                )
        );
    }

    public static void setTolerance(int tolerance){
        TOLERANCE = tolerance;
    }
}
