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

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/**
 * This 2016-2017 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 * <p>
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 * <p>
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 * <p>
 * This example assumes a "diamond" field configuration where the red and blue alliance stations
 * are adjacent on the corner of the field furthest from the audience.
 * From the Audience perspective, the Red driver station is on the right.
 * The two vision target are located on the two walls closest to the audience, facing in.
 * The Stones are on the RED side of the field, and the Chips are on the Blue side.
 * <p>
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name = "Just Park", group = "Autonomous")
public class JustParkAuton extends LinearOpMode {

    private Team6438ChassisHardwareMapCurrent robot = new Team6438ChassisHardwareMapCurrent();

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap);

        setUpMotors();

        waitForStart();

        while(opModeIsActive())
        {
            move(20, 20, .5);
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
        //Set zero power behavior
        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.tensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set runmode
        robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        motor.setTargetPosition(motor.getCurrentPosition() + (int)(dist * robot.hexCPI));
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

}
