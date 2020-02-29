package org.firstinspires.ftc.teamcode.markmattcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static android.os.SystemClock.sleep;

//@Disabled
@TeleOp(name = "Mark and Matt TeleOp", group = "test")
public class MattMarkTeleOp extends OpMode
{
    //hw map
    SimplifiedHardwareMap robot = new SimplifiedHardwareMap();

    //Variables
    int scaleFactor = 50;
    double strafeDec = .2;
    double slowSpeed = .2;
    double gamestickFactor = 2;
    int towerSize = 0;
    double liftUpSpeed = 1;
    int zeroTowerPositionOfSlides = 100; // need tweaking
    int ticksPerBlock = 100;            // need tweaking
    int triggerStrafeFactor = 100;

    @Override
    public void init()
    {
        //Init the hardware
        robot.init(hardwareMap);

        //Directions for the motors (THESE WORK)
        robot.FL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.FR.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.BL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.BR.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Use run using encoder to help speed control be more accurate
        robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.capstone.setPosition(.8);

        //Feedback for the user
        telemetry.speak("All systems go");
        telemetry.addData("Hardware Status: ","Mapped");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        //Needed for encoder movement;
        int positionToMoveUpBy;

        telemetry.addData("Current: Tower Height:", towerSize);
        telemetry.update();

        if (gamepad2.a)
        {
            robot.clampL.setPosition(.5);
            robot.clampR.setPosition(.5);
        }
        else
        {
            robot.clampL.setPosition(.2);
            robot.clampR.setPosition(.8);
        }


        if(gamepad2.dpad_up)
        {
            towerSize++;
            telemetry.speak("Tower Size Has been Increased, tower size now");
            telemetry.speak(String.valueOf(towerSize));
            telemetry.update();
            sleep(200);
        }
        else if (gamepad2.dpad_down)
        {
            towerSize = 0;
            telemetry.speak("Tower Size reset");
            telemetry.update();
            sleep(200);
        }
        else if(gamepad2.b)
        {
            goToTowerSize(towerSize);
        }
        if(gamepad2.right_stick_button)
        {
            robot.capstone.setPosition(.2);
        }
        else
        {
            robot.capstone.setPosition(.75);
        }
        if(gamepad2.left_bumper)
        {
            armMoveRelative(150);
        }
        else if (gamepad2.right_bumper)
        {
            armMoveRelative(-150);
        }
        else if (gamepad2.left_trigger > .05)
        {
            positionToMoveUpBy = (int) gamepad1.left_trigger * scaleFactor;
            armMoveRelative(positionToMoveUpBy);
        }
        else if (gamepad2.right_trigger > .05)
        {
            positionToMoveUpBy = (int) gamepad1.right_trigger * scaleFactor;
            armMoveRelative(-positionToMoveUpBy);
        }
        

        //This checks to see how we want the motors moved
        motorControl(gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.left_trigger,gamepad1.right_trigger);
        robot.rulerMotor.setPower(3*-gamepad2.left_stick_y/4);

    }
    /*
    * Moves to a position relative to current (- is lower, + is higher)
     */
    public void armMoveRelative(int positionRelative)
    {
        int newPosition = robot.liftMotor.getCurrentPosition() + positionRelative;

        robot.liftMotor.setPower(liftUpSpeed);

        robot.liftMotor.setTargetPosition(newPosition);

        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.liftMotor.isBusy())
        {
            telemetry.addData("Currently At", robot.liftMotor.getCurrentPosition());
            telemetry.addData("Going to", robot.liftMotor.getTargetPosition());
            motorControl(gamepad1.left_stick_y,gamepad1.right_stick_y,gamepad1.left_trigger,gamepad1.right_trigger);
            telemetry.update();
        }
    }

    /*
    * moves to an absolute position
     */
    public void armMoveAbsolute(int position)
    {
        robot.liftMotor.setPower(liftUpSpeed);

        robot.liftMotor.setTargetPosition(position);

        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.liftMotor.isBusy())
        {
            telemetry.addData("Currently At", robot.liftMotor.getCurrentPosition());
            telemetry.addData("Going to", robot.liftMotor.getTargetPosition());
            motorControl(gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.left_trigger,gamepad1.right_trigger);
            telemetry.update();
        }
    }
    /*
    * This method will control the motors, giving them power, DPAD 1 overrides the POV Power
     */
    public void motorControl (double leftJoystick, double rightJoystick, double leftTrigger, double rightTrigger)
    {
        double FLpower = 0, FRpower = 0, BLpower = 0, BRpower = 0;
        double drive = leftJoystick;
        double turn  =  -rightJoystick;

        // Combine drive and turn for robot.BLended motion.
        double leftPower  = drive + turn;
        double rightPower = drive - turn;

        // Normalize the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0)
        {
            leftPower /= max;
            rightPower /= max;
        }

        if(gamepad1.dpad_left)
        {
            FLpower = -strafeDec;
            FRpower = -strafeDec;
            BLpower = strafeDec;
            BRpower = strafeDec;
        }
        else if (gamepad1.dpad_right)
        {
            FLpower = strafeDec;
            FRpower = strafeDec;
            BLpower = -strafeDec;
            BRpower = -strafeDec;
        }
        if(gamepad1.dpad_down)
        {
            FLpower = -slowSpeed;
            FRpower = -slowSpeed;
            BLpower = -slowSpeed;
            BRpower = -slowSpeed;
        }
        else if (gamepad1.dpad_up)
        {
            FLpower = slowSpeed;
            FRpower = slowSpeed;
            BLpower = slowSpeed;
            BRpower = slowSpeed;
        }
        else if (gamepad1.back)
        {
            turnoffallMotors();
        }
        else if (leftTrigger>0.05)
        {
            driveStrafe(-leftTrigger);
        }
        else if(rightTrigger>0.05)
        {
            driveStrafe(rightTrigger);
        }
        else
        {
            FLpower = leftPower;
            FRpower = rightPower;
            BLpower = leftPower;
            BRpower = rightPower;
        }

        //Send the power
        robot.FL.setPower(FLpower/gamestickFactor);
        robot.FR.setPower(FRpower/gamestickFactor);
        robot.BL.setPower(BLpower/gamestickFactor);
        robot.BR.setPower(BRpower/gamestickFactor);
    }
    public void goToTowerSize (int towerSize)
    {
        int position = zeroTowerPositionOfSlides + (ticksPerBlock * towerSize);
        armMoveAbsolute(position);
    }
    //This one is pretty self explanatory
    public void turnoffallMotors()
    {
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.BL.setPower(0);
        robot.BR.setPower(0);
        robot.liftMotor.setPower(0);
    }

    public void driveStrafe(double trigger)
    {
        //Power
        double power = trigger * triggerStrafeFactor;

        //Opposite motors must have opposite power --needs tweaking
        robot.FL.setPower(power);
        robot.FR.setPower(-power);
        robot.BL.setPower(-power);
        robot.BR.setPower(power);

    }
}
