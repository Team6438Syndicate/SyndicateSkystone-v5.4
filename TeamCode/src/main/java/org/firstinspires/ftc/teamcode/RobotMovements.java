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
import org.firstinspires.ftc.teamcode.OpenCV.DogeCV.OurSkystoneDetector;
import org.jetbrains.annotations.NotNull;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class RobotMovements extends LinearOpMode
{
    protected Team6438ChassisHardwareMapCurrent robot = new Team6438ChassisHardwareMapCurrent();


    /**
     * @param hardwareMap map of the robot devices
     */
    protected void initRobot(HardwareMap hardwareMap) {
        robot.init(hardwareMap);
    }
    protected void initRobot(HardwareMap hardwareMap, boolean yes) {
        robot.init(hardwareMap,yes);
    }
    /**
     * Moving the intake
     *
     * @param direction direction of the wheels spinning
     */
    protected void intakeMove(@NotNull directions direction) {
        switch (direction) {
            case intake:
                motorSetPower(-1, robot.intakeLeft);
                motorSetPower(-1, robot.intakeRight);

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
    private void motorSetPower(double power, @NotNull DcMotor dcMotor) {
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

    public OpenCvCamera startOpenCV()
    {
        OpenCvCamera webcam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();

        webcam.pauseViewport();

        return webcam;
    }

    protected enum directions {intake, output}

}
