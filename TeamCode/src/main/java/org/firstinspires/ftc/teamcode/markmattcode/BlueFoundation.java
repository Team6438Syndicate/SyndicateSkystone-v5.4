package org.firstinspires.ftc.teamcode.markmattcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Test Foundation", group = "Concept")
public class BlueFoundation extends LinearOpMode
{
    SimplifiedHardwareMap robot = new SimplifiedHardwareMap();

    private double straightDrivePower = .25;
    private int ticksPerFullRot = 5500; //calculated
    private int criticalDistanceFar = 100;
    private int criticalDistanceClose = 200;
    private double slowstraightDrivePower = .25;
    private float initHeading;

    @Override
    public void runOpMode()
    {
        //Start the hardware map
        robot.init(hardwareMap);

        releaseFoundation();

        initHeading = robot.imu.getAngularOrientation().firstAngle;

        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //midFoundation();

        telemetry.addData("Hardware Status:", " Mapped");
        telemetry.speak("All Systems Go");
        telemetry.update();

        //WAITING FOR STARTING THE OPMODE
        waitForStart();

        //Loop while we're active
        while (opModeIsActive())
        {
            ////Strafe Left
            strafe(true, 18);

            midFoundation();

            //Go backward to right before foundation
            encoderFBslow(30);

            //checkAngle(0);

            telemetry.speak("lets go");

            grabFoundation();

            encoderFBslow(2);

            strafe(false, 12);

            encoderFBslow(-18);

            encoderTurn(false, 105);

            strafe(true, 12);

            encoderFBslow(8);

            elevatorStart(3);

            midFoundation();

            sleep(100);

            encoderFBslow(-37);

            //encoderTurn(true, 90);

            //rulerPark(30);

            stop();



            while (distanceTo() > criticalDistanceFar && distanceTo() < criticalDistanceClose)
                sleep(1000);

            //Grip foundation
            grabFoundation();

            //Back up a little more to fully grip foundation
            encoderFB(1);

            //Pull foundation back up to the wall
            encoderFB(-14);

            //Turn to move foundation into zone
            encoderTurn(false, 90);

            //Extend tape measure to park
            rulerPark(41);
        }
    }

    //Move forward or backwards (- is backwards, + is forwards)
    private void encoderFB(int inches) {
        //position
        robot.FL.setTargetPosition(robot.FL.getCurrentPosition() + robot.driveCPI * inches);
        robot.FR.setTargetPosition(robot.FL.getCurrentPosition() + robot.driveCPI * inches);
        robot.BL.setTargetPosition(robot.FL.getCurrentPosition() + robot.driveCPI * inches);
        robot.BR.setTargetPosition(robot.FL.getCurrentPosition() + robot.driveCPI * inches);

        robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //power
        robot.FL.setPower(straightDrivePower);
        robot.FR.setPower(straightDrivePower);
        robot.BL.setPower(straightDrivePower);
        robot.BR.setPower(straightDrivePower);

        while (opModeIsActive() && (robot.FL.isBusy() || robot.FR.isBusy() || robot.BL.isBusy() || robot.BR.isBusy()))
            ;

        while (opModeIsActive() && robot.FL.isBusy()) {
            telemetry.addData("FL ", "is busy");
            telemetry.update();
        }

        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.BL.setPower(0);
        robot.BR.setPower(0);
    }

    private void encoderFBslow(int inches) {
        //position
        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //
        robot.FL.setTargetPosition(robot.driveCPI * inches);
        robot.FR.setTargetPosition(robot.driveCPI * inches);
        robot.BL.setTargetPosition(robot.driveCPI * inches);
        robot.BR.setTargetPosition(robot.driveCPI * inches);

        //run to position
        robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //power
        robot.FL.setPower(slowstraightDrivePower);
        robot.FR.setPower(slowstraightDrivePower);
        robot.BL.setPower(slowstraightDrivePower);
        robot.BR.setPower(slowstraightDrivePower);

        //Loop to occupy
        while (opModeIsActive() && Math.abs(robot.FL.getCurrentPosition() - robot.FL.getTargetPosition()) > 50 || Math.abs(robot.FR.getCurrentPosition() - robot.FR.getTargetPosition()) > 50 || Math.abs(robot.BL.getCurrentPosition() - robot.BL.getTargetPosition()) > 50 || Math.abs(robot.BR.getCurrentPosition() - robot.BR.getTargetPosition()) > 50)//(robot.FL.isBusy() || robot.FR.isBusy() || robot.BL.isBusy() || robot.BR.isBusy()))
        {
            telemetry.addData("Current", robot.FL.getCurrentPosition());
            telemetry.addData("Heading", robot.FL.getTargetPosition());
            telemetry.update();
        }

        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.BL.setPower(0);
        robot.BR.setPower(0);
    }

    /*
    *Check if were normal to starting angle
     */
    private void checkAngle(double degrees)
    {
        while (Math.abs(robot.imu.getAngularOrientation().firstAngle - initHeading + degrees) > 3)
        {
            encoderTurn(false, robot.imu.getAngularOrientation().firstAngle - initHeading);
        }
        /*{
            robot.FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.FR.setPower(.25);
            robot.BR.setPower(.25);
            telemetry.speak("im moving");
        }*/
    }

    //Turning Method
    private void encoderTurn(boolean left, double degrees)
    {
        //This lets us determine how to move the wheels
        int encoderTicksToTurn = (int) ( (degrees / 360.0) * ticksPerFullRot);  //right??

        encoderTicksToTurn *= (left) ? -1 : 1;

        telemetry.addData("Ticks", encoderTicksToTurn);
        telemetry.update();
        sleep(100);

        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FL.setTargetPosition(-encoderTicksToTurn);
        robot.FR.setTargetPosition(encoderTicksToTurn);
        robot.BL.setTargetPosition(-encoderTicksToTurn);
        robot.BR.setTargetPosition(encoderTicksToTurn);


        robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.FL.setPower(slowstraightDrivePower);
        robot.FR.setPower(slowstraightDrivePower);
        robot.BL.setPower(slowstraightDrivePower);
        robot.BR.setPower(slowstraightDrivePower);

        while (Math.abs(robot.FL.getCurrentPosition()-robot.FL.getTargetPosition()) > 25)
        {
            telemetry.addData("Moving To", robot.FL.getTargetPosition());
            telemetry.addData("Currently At", robot.FL.getCurrentPosition());
            telemetry.update();
        }

        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.BL.setPower(0);
        robot.BR.setPower(0);

    }
    //Strafe Code may need redo
    private void strafe(boolean left, int inches)
    {
        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(left)
        {
            robot.FL.setTargetPosition(inches*robot.driveCPI);
            robot.FR.setTargetPosition(inches*robot.driveCPI);
            robot.BL.setTargetPosition(-inches*robot.driveCPI);
            robot.BR.setTargetPosition(-inches*robot.driveCPI);
        }
        else
        {
            robot.FL.setTargetPosition(-inches*robot.driveCPI);
            robot.FR.setTargetPosition(-inches*robot.driveCPI);
            robot.BL.setTargetPosition(inches*robot.driveCPI);
            robot.BR.setTargetPosition(inches*robot.driveCPI);
        }


        robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.FL.setPower(slowstraightDrivePower);
        robot.FR.setPower(slowstraightDrivePower);
        robot.BL.setPower(slowstraightDrivePower);
        robot.BR.setPower(slowstraightDrivePower);

        while (robot.FL.isBusy())
        {
            telemetry.addData("Moving To",  robot.FL.getTargetPosition());
            telemetry.addData("Currently At", robot.FL.getCurrentPosition());
            telemetry.update();
        }

        while (opModeIsActive() && (robot.FL.isBusy() || robot.FR.isBusy() || robot.BL .isBusy() || robot.BR.isBusy()));

        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.BL.setPower(0);
        robot.BR.setPower(0);
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

    private double distanceTo()
    {
        return robot.frontSensor.getDistance(DistanceUnit.MM);
    }

    private void rulerPark(double inches)
    {
        robot.rulerMotor.setTargetPosition((int)inches*(int)robot.rulerCPI);
        robot.rulerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rulerMotor.setPower(1);
        while (Math.abs(robot.rulerMotor.getCurrentPosition()-robot.rulerMotor.getTargetPosition()) > 20)
        {

        }
        robot.rulerMotor.setPower(0);
        robot.rulerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void elevatorStart(double inches)
    {
        robot.liftMotor.setTargetPosition((int)inches*robot.SlideCPI);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setPower(1);
        while(Math.abs(robot.liftMotor.getCurrentPosition() - robot.liftMotor.getTargetPosition()) > 50)
        {}

        robot.clampL.setPosition(.45);
        robot.clampR.setPosition(.55);

        sleep(500);

        robot.liftMotor.setTargetPosition(0);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setPower(.2);
        while(Math.abs(robot.liftMotor.getCurrentPosition() - robot.liftMotor.getTargetPosition()) > 50)
        {}

        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
