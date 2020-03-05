/*
 * Copyright (c) 2020.  [Bradley Abelman, Matthew Kaboolian, David Stekol]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

@Autonomous(name = "Just Park", group = "Autonomous")
public class JustParkAuton extends LinearOpMode {

    private Team6438ChassisHardwareMapCurrent robot = new Team6438ChassisHardwareMapCurrent();

    @Override
    public void runOpMode() throws InterruptedException
    {

        robot.init(hardwareMap);

        setUpMotors();

        waitForStart();

        while(opModeIsActive())
        {
            //move(20, 20, .5);
            tapeExtendByTime(41);
            stop();
        }
    }

    //Sets Direction, ZeroPowerBehavior, and Runmode for all motors
    public void setUpMotors()
    {
        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FR.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.BR.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rulerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //Set zero power behavior
        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rulerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.tensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set runmode
        robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rulerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.tensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Reset encoders
        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.tensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setTargetPosition(DcMotor motor, double dist)
    {
        motor.setTargetPosition(motor.getCurrentPosition() + (int)(dist));
    }

    public void setRunToPosition(DcMotor motor)
    {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setRunUsingEncoder(DcMotor motor)
    {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(DcMotor motor, double power)
    {
        motor.setPower(power);
    }

    public void move(double inchesL, double inchesR, double power)
    {
        inchesL *= robot.hexCPI;
        inchesR *= robot.hexCPI;

        setTargetPosition(robot.FL, inchesL);
        setTargetPosition(robot.FR, inchesR);
        setTargetPosition(robot.BL, inchesL);
        setTargetPosition(robot.BR, inchesR);

        setRunToPosition(robot.FL);
        setRunToPosition(robot.FR);
        setRunToPosition(robot.BL);
        setRunToPosition(robot.BR);

        setPower(robot.FL, power);
        setPower(robot.FR, power);
        setPower(robot.BL, power);
        setPower(robot.BR, power);

        while( FastMath.abs(robot.FL.getCurrentPosition() - robot.FL.getTargetPosition()) > 5 )
        {
            telemetry.addData("FL Position ", robot.FL.getCurrentPosition());
            telemetry.addData("FR Position ", robot.FR.getCurrentPosition());
            telemetry.addData("BL Position ", robot.BL.getCurrentPosition());
            telemetry.addData("BR Position ", robot.BR.getCurrentPosition());
            telemetry.addData("","");
            telemetry.addData("FL Target ", robot.FL.getTargetPosition());
            telemetry.addData("FR Target ", robot.FR.getTargetPosition());
            telemetry.addData("BL Target ", robot.BL.getTargetPosition());
            telemetry.addData("BR Target ", robot.BR.getTargetPosition());
            telemetry.update();
        }

        setRunUsingEncoder(robot.FL);
        setRunUsingEncoder(robot.FR);
        setRunUsingEncoder(robot.BL);
        setRunUsingEncoder(robot.BR);

        setPower(robot.FL, 0);
        setPower(robot.FR, 0);
        setPower(robot.BL, 0);
        setPower(robot.BR, 0);
    }

    /**
     * @param deg degree of turn
     * @param direction neg is left pos is right
     * @param power self explanatory
     */
    public void turn(double deg, int direction, double power)
    {
        int movementDist = FastMath.round((float)FastMath.toRadians(deg) * Team6438ChassisHardwareMapCurrent.radiusIN) * direction;

        move(movementDist, -movementDist, power);
    }

    public void tapeExtendByTime(double distance)
    {
        ElapsedTime time = new ElapsedTime();
        double waitTime = distance / .07543;

        setPower(robot.rulerMotor, 1);

        time.reset();
        while(time.milliseconds() < waitTime) //FastMath.abs(robot.rulerMotor.getCurrentPosition() - robot.rulerMotor.getTargetPosition()) > 5
        {
            telemetry.addData("Distance", distance);
            telemetry.addData("Remaining time", waitTime - time.milliseconds());
            telemetry.update();
        }

        setRunUsingEncoder(robot.rulerMotor);

        setPower(robot.rulerMotor, 0);

        telemetry.addData("Current Position ", robot.rulerMotor.getCurrentPosition());
        telemetry.update();
    }

    /**
     * Method to extend the tape measure using encoders
     * @param distance
     */
    public void tapeExtendByEncoder(double distance)
    {
        int positionToMove;

        positionToMove = (int) (distance * robot.rulerCPI);

        robot.rulerMotor.setTargetPosition(positionToMove);

        robot.rulerMotor.setPower(1);

        robot.rulerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Current Position ", robot.rulerMotor.getCurrentPosition());
        telemetry.addData("Going to", robot.rulerMotor.getTargetPosition());
        telemetry.update();

    }

}
