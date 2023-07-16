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

public class LeftAutoMidCycleCommand extends SequentialCommandGroup {
    private static int TOLERANCE = 10;

    public LeftAutoMidCycleCommand(Robot robot, int stackHeight){
        super(
          new SequentialCommandGroup(
                  //SCORE ON MEDIUM
                  new VerticalPositionCommand(robot.vertical, 980,0,5000),

                  new WaitUntilCommand(() -> robot.vertical.getPos() > stackHeight + 100),
                  new InstantCommand(() -> robot.intake.update(IntakeSubsystem.WristState.SLIGHT)),
                  new HorizontalPositionCommand(robot.horizontal,0.1),
                  new TurretPositionCommand(robot.turret,-2200,0,5000),
                  new WaitUntilCommand(() -> robot.turret.getPos() < -1600), //@TODO fix

                  //.andThen(new WaitCommand(500)),
                  new WaitUntilCommand(() -> robot.vertical.getAbsError() < TOLERANCE && robot.turret.controller.atGoal())
                          .andThen(new HorizontalPositionCommand(robot.horizontal, 0.32))
                          .andThen(new WaitCommand(200)),
                  new InstantCommand(() -> robot.intake.update(IntakeSubsystem.WristState.ACTIVE))
                          .andThen(new InstantCommand((() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN))))
                          .andThen(new WaitCommand(200))
                          .andThen(new HorizontalPositionCommand(robot.horizontal, 0.2))
                          .andThen(new WaitCommand(200)),

                  //INTAKE FROM STACK
                  new TurretPositionCommand(robot.turret,-820,0,5000),
                  new InstantCommand(() -> robot.intake.update(IntakeSubsystem.WristState.ACTIVE)),
                  new WaitUntilCommand(() -> robot.turret.getPos() > (-2160 + 200)),

                  new VerticalPositionCommand(robot.vertical, stackHeight,0,5000),
                  new WaitUntilCommand(() -> robot.vertical.getAbsError() < TOLERANCE && robot.turret.controller.atGoal()),
                  //swapped 42/43 with 45
                  new HorizontalPositionCommand(robot.horizontal,0.4).andThen(new WaitCommand(200)),


                  new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)).andThen(new WaitCommand(200))
          )
        );
    }

    public static void setTolerance(int tolerance){
        TOLERANCE = tolerance;
    }
}
