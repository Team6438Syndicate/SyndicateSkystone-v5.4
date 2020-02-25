package org.firstinspires.ftc.teamcode.markmattcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "Mark and Matt TeleOp", group = "test")
public class MattMarkTeleOp extends OpMode
{
    //Hardware
    Servo clampL, clampR;
    DcMotor liftMotor, FL,FR,BL,BR;

    //Variables
    int positionToMoveUpBy;
    int scaleFactor = 50;
    double strafeDec = .2;
    double slowSpeed = .2;
    double gamestickFactor = 1.75;
    double towerSize = 0;

    @Override
    public void init()
    {

     //hardware mapping
     clampL = hardwareMap.servo.get("clampL");
     clampR = hardwareMap.servo.get("clampR");

     //Motors
     liftMotor = hardwareMap.dcMotor.get("liftMotor");
     FL = hardwareMap.dcMotor.get("FL");
     FR = hardwareMap.dcMotor.get("FR");
     BL = hardwareMap.dcMotor.get("BL");
     BR = hardwareMap.dcMotor.get("BR");

     //Directions for the motors (THESE WORK)
     FL.setDirection(DcMotorSimple.Direction.FORWARD);
     FR.setDirection(DcMotorSimple.Direction.REVERSE);
     BL.setDirection(DcMotorSimple.Direction.FORWARD);
     BR.setDirection(DcMotorSimple.Direction.REVERSE);
     liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop()
    {
        //Needed for encoder movement;
        int positionToMoveUpBy;

        telemetry.addData("Current: Tower Height:", towerSize);

        if (gamepad1.a)
        {
            clampL.setPosition(.25);
            clampR.setPosition(.75);
        }
        else
        {
            clampL.setPosition(0.1);
            clampR.setPosition(.9);
        }

        if(gamepad2.dpad_up)
        {
            towerSize++;
        }


        if(gamepad1.left_bumper)
        {
            armMove(150);
        }
        else if (gamepad1.right_bumper)
        {
            armMove(-150);
        }
        else if (gamepad1.left_trigger > .05)
        {
            positionToMoveUpBy = (int) gamepad1.left_trigger * scaleFactor;
            armMove(positionToMoveUpBy);
        }
        else if (gamepad1.right_trigger > .05)
        {
            positionToMoveUpBy = (int) gamepad1.right_trigger * scaleFactor;
            armMove(-positionToMoveUpBy);
        }

        //This checks to see how we want the motors moved
        motorControl(gamepad1.left_stick_y,gamepad1.right_stick_y);

    }
    public void armMove(int position)
    {
        int newPosition = liftMotor.getCurrentPosition() + position;

        liftMotor.setPower(1);

        liftMotor.setTargetPosition(newPosition);

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (liftMotor.isBusy())
        {
            telemetry.addData("Currently At", liftMotor.getCurrentPosition());
            telemetry.addData("Going to", liftMotor.getTargetPosition());
            motorControl(gamepad1.left_stick_y,gamepad1.right_stick_y);
            telemetry.update();
        }
    }
    /*
    * This method will control the motors, giving them power, DPAD 1 overrides the POV Power
     */
    public void motorControl (double leftJoystick, double rightJoystick)
    {
        double FLpower = 0, FRpower = 0, BLpower = 0, BRpower = 0;
        double drive = -leftJoystick;
        double turn  =  -rightJoystick;

        // Combine drive and turn for blended motion.
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
        else
        {
            FLpower = leftPower;
            FRpower = rightPower;
            BLpower = leftPower;
            BRpower = rightPower;
        }

        //Send the power
        FL.setPower(FLpower);
        FR.setPower(FRpower);
        BL.setPower(BLpower);
        BR.setPower(BRpower);
    }

    //This one is pretty self explanatory
    public void turnoffallMotors()
    {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        liftMotor.setPower(0);
    }
}
