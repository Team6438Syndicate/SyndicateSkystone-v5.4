package org.firstinspires.ftc.teamcode.markmattcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Test Blue (Left) Foundation", group = "Concept")
public class BlueFoundation extends LinearOpMode
{
    SimplifiedHardwareMap robot = new SimplifiedHardwareMap();

    private double straightDrivePower = .25;
    private int ticksPerFullRot = 5500; //calculated

    //private int criticalDistanceFar = 100;
    //private int criticalDistanceClose = 200;

    private float initHeading;
    private double clampClosedL = .45;
    private double clampClosedR = .55;

    @Override
    public void runOpMode()
    {
        //Start the hardware map
        robot.init(hardwareMap);

        //release the servos to prevent unintended actions
        releaseFoundation();

        //grab our heading at start
        initHeading = robot.imu.getAngularOrientation().firstAngle;

        //reset the encoders
        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Brake power
        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Telem stuff
        telemetry.addData("Hardware Status:", " Mapped");
        telemetry.speak("All Systems Go");
        telemetry.update();

        //WAITING FOR STARTING THE OPMODE
        waitForStart();

        //Loop while we're active
        while (opModeIsActive())
        {
            ////Strafe Left
            strafe(false, 18,.25);

            //Move the servos ready to engage
            midFoundation();

            //Go backward to right before foundation
            encoderFB(30, .25);

            //checkAngle(0);

            //Rewarding Message
            telemetry.speak("Let's go");

            //Grab it
            grabFoundation();

            //Head forward a little to ensure full engagement
            encoderFB(2, .25);

            //Strafe
            strafe(true, 12,.25);

            //Slow encoder
            encoderFB(-18, .25);

            //Turn
            encoderTurn(true, 105, .25);
            strafe(false, 12,1);

            encoderFB(8, .25);

            elevatorStart(3,1,.3);

            midFoundation();

            sleep(100);

            encoderFB(-37, .25);

            //encoderTurn(true, 90);

            //rulerPark(30);

            //end the op mode
            stop();

        }
    }

    /*
     * Move forward or backwards (- is backwards, + is forwards)
     */
    private void encoderFB(int inches, double power) {
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
        robot.FL.setPower(power);
        robot.FR.setPower(power);
        robot.BL.setPower(power);
        robot.BR.setPower(power);

        //Loop to occupy
        while (opModeIsActive() && Math.abs(robot.FL.getCurrentPosition() - robot.FL.getTargetPosition()) > 50 || Math.abs(robot.FR.getCurrentPosition() - robot.FR.getTargetPosition()) > 50 || Math.abs(robot.BL.getCurrentPosition() - robot.BL.getTargetPosition()) > 50 || Math.abs(robot.BR.getCurrentPosition() - robot.BR.getTargetPosition()) > 50)//(robot.FL.isBusy() || robot.FR.isBusy() || robot.BL.isBusy() || robot.BR.isBusy()))
        {
            telemetry.addData("Current", robot.FL.getCurrentPosition());
            telemetry.addData("Heading", robot.FL.getTargetPosition());
            telemetry.update();
        }

        //kill power
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
            encoderTurn(false, robot.imu.getAngularOrientation().firstAngle - initHeading,.25);
        }
    }

    //Turning Method
    private void encoderTurn(boolean left, double degrees, double power)
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

        robot.FL.setPower(power);
        robot.FR.setPower(power);
        robot.BL.setPower(power);
        robot.BR.setPower(power);

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
    private void strafe(boolean left, int inches, double power)
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

        robot.FL.setPower(power);
        robot.FR.setPower(power);
        robot.BL.setPower(power);
        robot.BR.setPower(power);

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

    /*
    * Ruler Park
     */
    private void rulerPark(double inches, double power)
    {
        robot.rulerMotor.setTargetPosition((int)inches*(int)robot.rulerCPI);
        robot.rulerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rulerMotor.setPower(power);
        while (Math.abs(robot.rulerMotor.getCurrentPosition()-robot.rulerMotor.getTargetPosition()) > 20)
        {

        }
        robot.rulerMotor.setPower(0);
        robot.rulerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void elevatorStart(double inches, double upPower, double downPower)
    {
        robot.liftMotor.setTargetPosition((int)inches*robot.SlideCPI);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setPower(upPower);
        while(Math.abs(robot.liftMotor.getCurrentPosition() - robot.liftMotor.getTargetPosition()) > 50)
        {}

        robot.clampL.setPosition(clampClosedL);
        robot.clampR.setPosition(clampClosedR);

        sleep(500);

        robot.liftMotor.setTargetPosition(0);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setPower(downPower);
        while(Math.abs(robot.liftMotor.getCurrentPosition() - robot.liftMotor.getTargetPosition()) > 50)
        {}

        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
