/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.markmattcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@TeleOp(name = "Matt B Auton", group = "Concept")
@Disabled
public class BlueSample extends LinearOpMode
{
    //HardwareMap
    SimplifiedHardwareMap robot = new SimplifiedHardwareMap();

    //Vars
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private final String VUFORIA_KEY = SimplifiedHardwareMap.VUFORIA_KEY;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private double straightDrivePower = 1;
    private int singleStackPosition = 100;
    private int ticksPerFullRot = 10000; //calculated

    @Override
    public void runOpMode()
    {
        //Start the hardware map
        robot.init(hardwareMap);

        //Make the motors run to position
        robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Start our computer vision
        initCV();

        //start the tfod
        tfod.activate();

        //Telemetry to let user know we're ready to go
        telemetry.addData("Hardware Status:", " Mapped");
        telemetry.speak("All Systems Go");
        telemetry.update();

        //WAITING FOR STARTING THE OPMODE
        waitForStart();

        //Loop while we're active
        while (opModeIsActive())
        {
            if (tfod != null)
            {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null)
                {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions)
                    {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                    }
                    telemetry.update();
                }
            }
        }

        if (tfod != null)
        {
            tfod.shutdown();
        }
    }

    //Start Computer Vision Protocols
    private void initCV()
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    //Move forward or backwards (- is backwards, + is forwards)
    private void encoderFB(int inches)
    {
        //position
        robot.FL.setTargetPosition(robot.driveCPI*inches);
        robot.FR.setTargetPosition(robot.driveCPI*inches);
        robot.BL.setTargetPosition(robot.driveCPI*inches);
        robot.BR.setTargetPosition(robot.driveCPI*inches);

        //power
        robot.FL.setPower(straightDrivePower);
        robot.FR.setPower(straightDrivePower);
        robot.BL.setPower(straightDrivePower);
        robot.BR.setPower(straightDrivePower);
    }
    //Turning Method
    private void encoderTurn(boolean left, int degrees)
    {
        //This lets us determine how to move the wheels
        int encoderTicksToTurn = degrees * (1 / (360 * ticksPerFullRot));  //right??

        if(left)
        {
            robot.FL.setTargetPosition(-encoderTicksToTurn);
            robot.FR.setTargetPosition(encoderTicksToTurn);
            robot.BL.setTargetPosition(-encoderTicksToTurn);
            robot.BR.setTargetPosition(encoderTicksToTurn);
        }
        else
        {
            robot.FL.setTargetPosition(encoderTicksToTurn);
            robot.FR.setTargetPosition(-encoderTicksToTurn);
            robot.BL.setTargetPosition(encoderTicksToTurn);
            robot.BR.setTargetPosition(-encoderTicksToTurn);
        }
    }
    //Strafe Code may need redo
    private void strafe(boolean left, int inches)
    {
        if(left)
        {
            robot.FL.setTargetPosition(inches*robot.driveCPI);
            robot.FR.setTargetPosition(-inches*robot.driveCPI);
            robot.BL.setTargetPosition(inches*robot.driveCPI);
            robot.BR.setTargetPosition(-inches*robot.driveCPI);
        }
        else
        {
            robot.FL.setTargetPosition(-inches*robot.driveCPI);
            robot.FR.setTargetPosition(inches*robot.driveCPI);
            robot.BL.setTargetPosition(-inches*robot.driveCPI);
            robot.BR.setTargetPosition(inches*robot.driveCPI);
        }
    }
}
