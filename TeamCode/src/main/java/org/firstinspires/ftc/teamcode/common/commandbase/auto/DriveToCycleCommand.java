package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class DriveToCycleCommand extends CommandBase {
    private Robot robot;
    private ElapsedTime timer;

    public double headingOffset = 0;
    public double robotHeading = 0;
    public double headingError  = 0;
    public double headingGoal = 0;
    public double encoderOffset = 0;

    public DriveToCycleCommand(Robot robot){
        this.robot = robot;
        timer = new ElapsedTime();

    }

    @Override
    public void initialize(){
        encoderOffset = robot.drive.encoderTicksToInches(robot.drive.getWheelPosition());
        headingOffset = robot.drive.getAngle();
        robotHeading = 0;
        timer.reset();
    }

    @Override
    public void execute(){
        robotHeading = robot.drive.getAngle() - headingOffset;
        headingError = robotHeading - headingGoal;


        double currPos = robot.drive.encoderTicksToInches(robot.drive.getWheelPosition()) - encoderOffset;
        double fwdPower = Range.clip(0.08 * (50 - currPos), -0.5, 0.5);
        double rotPower = Range.clip(4.583662 * (headingError),-0.5,0.5);

        robot.drive.setMotorPowers(fwdPower + rotPower,fwdPower + rotPower,fwdPower - rotPower,fwdPower - rotPower);

    }

    @Override
    public boolean isFinished(){
        return timer.seconds() > 5;
        //max 5 seconds
    }
}
