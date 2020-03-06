package org.firstinspires.ftc.teamcode.markmattcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Test Foundation", group = "Concept")
public class BlueFoundation extends LinearOpMode {
    SimplifiedHardwareMap robot = new SimplifiedHardwareMap();

    private double straightDrivePower = .75;
    private int ticksPerFullRot = 10000; //calculated
    private int criticalDistanceFar = 100;
    private int criticalDistanceClose = 200;
    private double slowstraightDrivePower = .25;
    private float initHeading;

    @Override
    public void runOpMode() {
        //Start the hardware map
        robot.init(hardwareMap);
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

        midFoundation();

        telemetry.addData("Hardware Status:", " Mapped");
        telemetry.speak("All Systems Go");
        telemetry.update();

        //WAITING FOR STARTING THE OPMODE
        waitForStart();

        //Loop while we're active
        while (opModeIsActive()) {

            //Strafe Left
            strafe(true, 14);

            //Go backward to right before foundation
            encoderFBslow(30);
            //encoderFB(18);

            checkAngle();

            telemetry.speak("lets go");

            grabFoundation();

            encoderFBslow(2);

            encoderFBslow(-25);

            encoderTurn(false, 90);

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

            //Expand elevator to get ready of TeleOp
            elevatorExpand(12);
        }
    }

    //Move forward or backwards (- is backwards, + is forwards)
    private void encoderFB(int inches) {
        //position
        robot.FL.setTargetPosition(robot.driveCPI * inches);
        robot.FR.setTargetPosition(robot.driveCPI * inches);
        robot.BL.setTargetPosition(robot.driveCPI * inches);
        robot.BR.setTargetPosition(robot.driveCPI * inches);

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
        while (opModeIsActive() && (robot.FL.isBusy() || robot.FR.isBusy() || robot.BL.isBusy() || robot.BR.isBusy()))


        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.BL.setPower(0);
        robot.BR.setPower(0);
    }

    /*
    *Check if were normal to starting agnle
     */
    private void checkAngle()
    {
        while( robot.imu.getAngularOrientation().firstAngle > (initHeading) )
        {
            robot.FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.FR.setPower(.25);
            robot.BR.setPower(.25);
            telemetry.speak("im moving");
        }
    }

    //Turning Method
    private void encoderTurn(boolean left, int degrees)
    {
        //This lets us determine how to move the wheels
        int encoderTicksToTurn = degrees * (1 / (360 * ticksPerFullRot));  //right??

        if (left)
        {
            robot.FL.setTargetPosition(-encoderTicksToTurn);
            robot.FR.setTargetPosition(encoderTicksToTurn);
            robot.BL.setTargetPosition(-encoderTicksToTurn);
            robot.BR.setTargetPosition(encoderTicksToTurn);
            while (robot.FL.isBusy() && robot.FR.isBusy() && robot.BL.isBusy() && robot.BR.isBusy())
            {
                telemetry.addData("Moving To", robot.FL.getTargetPosition());
                telemetry.addData("Currently At", robot.FL.getCurrentPosition());
                telemetry.update();
            }
        }
        else
        {
            robot.FL.setTargetPosition(encoderTicksToTurn);
            robot.FR.setTargetPosition(-encoderTicksToTurn);
            robot.BL.setTargetPosition(encoderTicksToTurn);
            robot.BR.setTargetPosition(-encoderTicksToTurn);
        }
        robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.FL.setPower(1);
        robot.FR.setPower(1);
        robot.BL.setPower(1);
        robot.BR.setPower(1);
        while (robot.FL.isBusy())
        {
            telemetry.addData("Moving To", robot.FL.getTargetPosition());
            telemetry.addData("Currently At", robot.FL.getCurrentPosition());
            telemetry.update();
        }
        while (opModeIsActive() && (robot.FL.isBusy() || robot.FR.isBusy() || robot.BL.isBusy() || robot.BR.isBusy())) ;

            robot.FL.setPower(0);
            robot.FR.setPower(0);
            robot.BL.setPower(0);
            robot.BR.setPower(0);

    }
    //Strafe Code may need redo
    private void strafe(boolean left, int inches)
    {
        if(left)
        {
            robot.FL.setTargetPosition(-inches*robot.driveCPI);
            robot.FR.setTargetPosition(inches*robot.driveCPI);
            robot.BL.setTargetPosition(inches*robot.driveCPI);
            robot.BR.setTargetPosition(-inches*robot.driveCPI);
        }
        else
        {
            robot.FL.setTargetPosition(inches*robot.driveCPI);
            robot.FR.setTargetPosition(-inches*robot.driveCPI);
            robot.BL.setTargetPosition(-inches*robot.driveCPI);
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
    }

    private void elevatorExpand(double inches)
    {
        robot.liftMotor.setTargetPosition((int)inches*(int)robot.SlideCPI);
    }
}
