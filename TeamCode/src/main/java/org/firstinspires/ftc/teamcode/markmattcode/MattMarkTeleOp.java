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
public class MattMarkTeleOp extends OpMode {
    Servo clampL, clampR;
    DcMotor liftMotor, FL,FR,BL,BR;
    int positionToMoveUpBy;
    int scaleFactor = 50;
    double strafeDec = .2;
    double slowSpeed = .2;
    double gamestickFactor = 1.75;
    @Override
    public void init()
    {
     clampL = hardwareMap.servo.get("clampL");
     clampR = hardwareMap.servo.get("clampR");
     liftMotor = hardwareMap.dcMotor.get("liftMotor");
     FL = hardwareMap.dcMotor.get("FL");
     FR = hardwareMap.dcMotor.get("FR");
     BL = hardwareMap.dcMotor.get("BL");
     BR = hardwareMap.dcMotor.get("BR");


     FL.setDirection(DcMotorSimple.Direction.FORWARD);
     FR.setDirection(DcMotorSimple.Direction.REVERSE);
     BL.setDirection(DcMotorSimple.Direction.FORWARD);
     BR.setDirection(DcMotorSimple.Direction.REVERSE);

     liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop()
    {
        positionToMoveUpBy = 0;

        double FLpower = -gamepad1.left_stick_y/gamestickFactor;
        double FRpower = -gamepad1.right_stick_y/gamestickFactor;
        double BLpower = -gamepad1.left_stick_y/gamestickFactor;
        double BRpower = -gamepad1.right_stick_y/gamestickFactor;
        telemetry.addData("LiftMotor", liftMotor.getCurrentPosition());
        telemetry.addData("LiftMotor Power", liftMotor.getPower());
        telemetry.update();

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

        FL.setPower (FLpower);
        FR.setPower (FRpower);
        BL.setPower (BLpower);
        BR.setPower (BRpower);
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
            turnoffallMotors();
            telemetry.update();
        }
    }
    public void turnoffallMotors()
    {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

    }
}
