package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.HorizontalPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.TurretPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.VerticalPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class AutoMidCycleCommand extends SequentialCommandGroup {
    private static int TOLERANCE = 30;

    public AutoMidCycleCommand(Robot robot){
        super(
          new SequentialCommandGroup(
                  //SCORE ON MEDIUM
                  new VerticalPositionCommand(robot.vertical, 2020,0,5000),
                  new WaitUntilCommand(() -> robot.vertical.getPos() > 250),

                  new TurretPositionCommand(robot.turret,-2100,0,5000),
                  new WaitUntilCommand(() -> robot.turret.getPos() < -1100), //@TODO fix

                  new HorizontalPositionCommand(robot.horizontal,0.2).andThen(new WaitCommand(500)),
                  new WaitUntilCommand(() -> robot.vertical.getAbsError() > TOLERANCE && robot.turret.getAbsError() > TOLERANCE),

                  new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)).andThen(new WaitCommand(500)),

                  //INTAKE FROM STACK
                  new TurretPositionCommand(robot.turret,-850,0,5000),
                  new WaitUntilCommand(() -> robot.turret.getPos() > -1200),

                  new HorizontalPositionCommand(robot.horizontal,0.4).andThen(new WaitCommand(500)),
                  new VerticalPositionCommand(robot.vertical, 630,0,5000),
                  new WaitUntilCommand(() -> robot.vertical.getAbsError() > TOLERANCE && robot.turret.getAbsError() > TOLERANCE),

                  new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)).andThen(new WaitCommand(500))
          )
        );
    }

    public static void setTolerance(int tolerance){
        TOLERANCE = tolerance;
    }
}
