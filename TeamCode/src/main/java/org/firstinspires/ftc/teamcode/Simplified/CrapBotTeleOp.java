package org.firstinspires.ftc.teamcode.Simplified;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp(name = "Crap Bot Tele", group = "z")
public class CrapBotTeleOp extends OpMode
{
    DcMotor leftMotor,rightMotor;

    @Override
    public void init()
    {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Hardware Status", "Mapped");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        leftMotor.setPower(-gamepad1.left_stick_y);
        rightMotor.setPower(-gamepad1.right_stick_y);

        telemetry.addData("Left Motor", leftMotor.getPower());
        telemetry.addData("Right Motor", rightMotor.getPower());
        telemetry.update();

    }
}
