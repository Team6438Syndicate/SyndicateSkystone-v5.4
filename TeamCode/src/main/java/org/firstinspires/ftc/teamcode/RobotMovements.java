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

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.jetbrains.annotations.NotNull;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class RobotMovements extends LinearOpMode
{
    protected Team6438ChassisHardwareMapCurrent robot = new Team6438ChassisHardwareMapCurrent();


    /**
     * Two Servo movements
     *
     * @param A1        Button 1
     * @param B1        Button 2
     * @param position1 Position for Button 1
     * @param position2 Position for Button 2
     * @param servo1    One servo
     * @param servo2    Another servo
     */
    protected void servoMovement(boolean A1, boolean B1, double position1, double position2, Servo servo1, Servo servo2) {

        if (A1) {
            servo1.setPosition(position1);
            servo2.setPosition(position1);

        } else if (B1) {
            servo1.setPosition(position2);
            servo2.setPosition(position2);

        }
    }

    /**
     * @param hardwareMap map of the robot devices
     */
    protected void initRobot(HardwareMap hardwareMap) {
        robot.init(hardwareMap);
    }

    /**
     * One Servo movements
     *
     * @param A1        Button 1
     * @param B1        Button 2
     * @param position1 Position for Button 1
     * @param position2 Position for Button 2
     * @param servo1    One servo
     */
    protected void servoMovement(boolean A1, boolean B1, double position1, double position2, Servo servo1) {

        if (A1) {
            servo1.setPosition(position1);

        } else if (B1) {
            servo1.setPosition(position2);

        }
    }

    /**
     * Moving the intake
     *
     * @param direction direction of the wheels spinning
     */
    protected void intakeMove(@NotNull directions direction) {
        switch (direction) {
            case intake:
                motorSetPower(- 1, robot.intakeLeft);
                motorSetPower(- 1, robot.intakeRight);

            case output:
                motorSetPower(1, robot.intakeLeft);
                motorSetPower(1, robot.intakeRight);

        }
    }

    /**
     * Normal operation of spinning motors
     *
     * @param power   power level of the motors
     * @param dcMotor the direct current motor to be controlled
     */
    protected void motorSetPower(double power, @NotNull DcMotor dcMotor) {
        dcMotor.setPower(power);
    }

    /**
     * Human control of the intake
     *
     * @param X1 one button for one direction
     * @param Y1 one button for another direction
     */
    protected void hControlIntakeMove(boolean X1, boolean Y1) {
        if (X1)// put out
        {
            intakeMove(directions.output);
        } else if (Y1) //take in
        {
            intakeMove(directions.intake);
        }
    }

    /**
     * Encoder driven motors
     *
     * @param position position to go to
     * @param power    power level of the motors
     * @param motor    the direct current motor to be controlled
     */
    protected void motorMovementEncoder(int position, double power, @NotNull DcMotor motor) {

        int currentPosition = motor.getCurrentPosition();

        motor.setTargetPosition(currentPosition + position);
        motor.setPower(power);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    public Locations detectSkystone(boolean isRed)
    {
        SkystoneDetector skyStoneDetector;

        OpenCvCamera webcam;

        Locations position = Locations.Close;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();

        skyStoneDetector = new SkystoneDetector();
        webcam.setPipeline(skyStoneDetector);

        webcam.startStreaming(160, 120, OpenCvCameraRotation.UPSIDE_DOWN);

        while (!isStarted())
        {
            if (isRed)
            {
                if (skyStoneDetector.getScreenPosition().x < 150)
                {
                    position = Locations.Close;
                }
                else if (skyStoneDetector.getScreenPosition().x > 150 && skyStoneDetector.getScreenPosition().x < 200)
                {
                    position = Locations.Center;
                }
                else
                {
                    position = Locations.Far;
                }
            }
            else
            {
                if (skyStoneDetector.getScreenPosition().x < 50)
                {
                    position = Locations.Far;
                }
                else if (skyStoneDetector.getScreenPosition().x > 55 && skyStoneDetector.getScreenPosition().x < 83)
                {
                    position = Locations.Center;
                }
                else
                {
                    position = Locations.Close;
                }
            }
        }

        telemetry.speak(position.toString());

        return position;
    }

    public enum Locations {Close, Center, Far}

    private enum conditionCode {coord, angle}

    private enum coordCondition {returnFirst, returnSecond, returnAverage, rerunError}

    protected enum directions {intake, output}


}
