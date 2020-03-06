package org.firstinspires.ftc.teamcode.markmattcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Team6438AutonomousBlue;
import org.firstinspires.ftc.teamcode.Team6438ChassisHardwareMapCurrent;

@Autonomous(name = "Distance Readout", group = "Test")
public class Distance extends LinearOpMode
{
    SimplifiedHardwareMap robot = new SimplifiedHardwareMap();
    @Override
    public void runOpMode() throws InterruptedException
    {
     telemetry.addData(String.valueOf(distanceTo()), "MM");
    }
    private double distanceTo()
    {
        return robot.frontSensor.getDistance(DistanceUnit.MM);
    }
}
