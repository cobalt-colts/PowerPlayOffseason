package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class TestUltrasonic extends LinearOpMode {

    public AnalogInput distanceSensor;
    private double lastVoltage = 0;

    @Override
    public  void runOpMode(){
        distanceSensor = hardwareMap.get(AnalogInput.class, "distanceSensor");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Current Distance: ", getRawDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }

    public double getRawDistance(DistanceUnit distanceUnit) {
        double rawVoltage = distanceSensor.getVoltage();

        if (rawVoltage > 1.4823) { //1.4823 volts is about 144 inches, the width of the field.  If the volate is greater then this, there must have been a faulty reading
            rawVoltage = lastVoltage; //use last distance instead of new one.
        } else {
            lastVoltage = rawVoltage;
        }

        //Constants gotten from getting the true distance from distance sensor to wall via tape measure,
        //then (distance/voltage) got the values in inches, then converted that to other distance units
        if (distanceUnit == DistanceUnit.CM) return rawVoltage*  542.1822921180930552;
        if (distanceUnit == DistanceUnit.METER) return rawVoltage * 5.421822921180930552;
        if (distanceUnit == DistanceUnit.MM) return rawVoltage * 5421.822921180930552;
        if (distanceUnit == DistanceUnit.INCH) return rawVoltage * 213.45759532208388;
        return Double.NaN;
    }
}
