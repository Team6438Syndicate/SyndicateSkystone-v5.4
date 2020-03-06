package org.firstinspires.ftc.teamcode.markmattcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Matt B Auton", group = "Concept")
public class RedFoundation extends LinearOpMode
{
    SimplifiedHardwareMap robot = new SimplifiedHardwareMap();

    private double straightDrivePower = 1;
    private int ticksPerFullRot = 10000; //calculated

    @Override
    public void runOpMode()
    {
        //Start the hardware map
        robot.init(hardwareMap);

        //Make the motors run to position
        robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Hardware Status:", " Mapped");
        telemetry.speak("All Systems Go");
        telemetry.update();

        //WAITING FOR STARTING THE OPMODE
        waitForStart();

        //Loop while we're active
        while (opModeIsActive())
        {
            //Go backward to right before foundation
            encoderFB(-12);

            //Mid foundation position
            midFoundation();

            //Back up to foundation
            encoderFB(-1);

            //Grip foundation
            grabFoundation();

            //Back up a little more to fully grip foundation
            encoderFB(-1);

            //Pull foundation back up to the wall
            encoderFB(14);

            //Turn to move foundation into zone
            encoderTurn(false, 90);

            //Extend tape measure to park
            rulerPark(41);
        }
    }

    //Move forward or backwards (- is backwards, + is forwards)
    private void encoderFB(int inches)
    {
        //position
        robot.FL.setTargetPosition(robot.driveCPI*inches);
        robot.FR.setTargetPosition(robot.driveCPI*inches);
        robot.BL.setTargetPosition(robot.driveCPI*inches);
        robot.BR.setTargetPosition(robot.driveCPI*inches);

        //power
        robot.FL.setPower(straightDrivePower);
        robot.FR.setPower(straightDrivePower);
        robot.BL.setPower(straightDrivePower);
        robot.BR.setPower(straightDrivePower);
    }

    //Turning Method
    private void encoderTurn(boolean left, int degrees)
    {
        //This lets us determine how to move the wheels
        int encoderTicksToTurn = degrees * (1 / (360 * ticksPerFullRot));  //right??

        if(left)
        {
            robot.FL.setTargetPosition(-encoderTicksToTurn);
            robot.FR.setTargetPosition(encoderTicksToTurn);
            robot.BL.setTargetPosition(-encoderTicksToTurn);
            robot.BR.setTargetPosition(encoderTicksToTurn);
        }
        else
        {
            robot.FL.setTargetPosition(encoderTicksToTurn);
            robot.FR.setTargetPosition(-encoderTicksToTurn);
            robot.BL.setTargetPosition(encoderTicksToTurn);
            robot.BR.setTargetPosition(-encoderTicksToTurn);
        }
    }

    //Strafe Code may need redo
    private void strafe(boolean left, int inches)
    {
        if(left)
        {
            robot.FL.setTargetPosition(inches*robot.driveCPI);
            robot.FR.setTargetPosition(-inches*robot.driveCPI);
            robot.BL.setTargetPosition(inches*robot.driveCPI);
            robot.BR.setTargetPosition(-inches*robot.driveCPI);
        }
        else
        {
            robot.FL.setTargetPosition(-inches*robot.driveCPI);
            robot.FR.setTargetPosition(inches*robot.driveCPI);
            robot.BL.setTargetPosition(-inches*robot.driveCPI);
            robot.BR.setTargetPosition(inches*robot.driveCPI);
        }
    }

    private void grabFoundation()
    {
        robot.foundationL.setPosition(1);
        robot.foundationR.setPosition(0);
    }

    private void releaseFoundation()
    {
        robot.foundationL.setPosition(.5);
        robot.foundationR.setPosition(.5);
    }

    private void midFoundation()
    {
        robot.foundationL.setPosition(.9);
        robot.foundationR.setPosition(.1);
    }

    private void rulerPark(double inches)
    {
        robot.rulerMotor.setTargetPosition((int)inches*(int)robot.rulerCPI);
    }
}
