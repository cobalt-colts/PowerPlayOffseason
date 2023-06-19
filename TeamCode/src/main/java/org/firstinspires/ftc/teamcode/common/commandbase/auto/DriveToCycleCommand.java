package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

@Config
public class DriveToCycleCommand extends CommandBase {
    File file;
    PrintWriter pw;

    private Robot robot;
    private Telemetry telemetry;
    private ElapsedTime timer;

    public static PIDController fwd,rot, lat;
    public static PIDCoefficients fwdVal = new PIDCoefficients(), rotVal = new PIDCoefficients(), latVal = new PIDCoefficients();


    public double headingOffset = 0;
    public double robotHeading = 0;
    public double headingError  = 0;
    private double fwdWheelOffset = 0;
    private double latWheelOffset = 0;


    public DriveToCycleCommand(Robot robot, Telemetry telemetry, double headingOffset, double fwdWheelOffset, double latWheelOffset){
        this.robot = robot;
        timer = new ElapsedTime();
        this.telemetry = telemetry;
        this.headingOffset = headingOffset;

        this.fwdWheelOffset = fwdWheelOffset;
        this.latWheelOffset = latWheelOffset;


        robotHeading = 0;
    }

    @Override
    public void initialize(){
        fwd = new PIDController(fwdVal.p, fwdVal.i, fwdVal.d);
        rot = new PIDController(rotVal.p, rotVal.i, rotVal.d);
        lat = new PIDController(latVal.p, latVal.i, latVal.d);

        timer.reset();
    }

    @Override
    public void execute(){
        fwd.setPID(fwdVal.p, fwdVal.i,fwdVal.d);
        rot.setPID(rotVal.p,rotVal.i,rotVal.d);
        lat.setPID(latVal.p,latVal.i,latVal.d);

        robotHeading = robot.drive.getAngle() - headingOffset;
        headingError = -robotHeading;

        while(headingError > Math.PI){
            headingError -= 2 * Math.PI;
        }

        while(headingError < -Math.PI){
            headingError += 2 * Math.PI;
        }

        double fwdPos = robot.drive.getForwardPosition() - fwdWheelOffset;
        double latPos = robot.drive.getLateralPosition() - latWheelOffset;

        double fwdPower = Range.clip(fwd.calculate(fwdPos,52), -0.5, 0.5) + 0.0 * Math.signum(fwd.calculate(fwdPos,52));
        double rotPower = Range.clip(rot.calculate(headingError, 0),-0.5,0.5) + 0.0 * Math.signum(rot.calculate(headingError, 0));
        double latPower = Range.clip(lat.calculate(latPos,0), -0.5, 0.5) + 0.0 * Math.signum(lat.calculate(latPos,0));

        robot.drive.setMotorPowers(fwdPower + rotPower - latPower,fwdPower + rotPower + latPower,fwdPower - rotPower - latPower,fwdPower - rotPower + latPower);

        telemetry.addData("heading angle", robotHeading);
        telemetry.addData("currFwd: ",fwdPos);
        telemetry.addData("currStr: ",latPos);




    }

    public double getHeadingOffset() {
        return headingOffset;
    }

    public double getFwdWheelOffset() {
        return fwdWheelOffset;
    }

    public double getLatWheelOffset() {
        return latWheelOffset;
    }

    @Override
    public boolean isFinished(){

        return false;
        //max 5 seconds
    }
}
