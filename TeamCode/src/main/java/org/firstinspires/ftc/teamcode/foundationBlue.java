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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.odometry.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;

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
@Disabled
@Autonomous(name = "Blue Foundation", group = "Autonomous")
public class foundationBlue extends RobotMovements {

    /**5
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {

        //init the hardware
        initRobot(hardwareMap);
        //Telemetry
        telemetry.addData("Hardware Status:", "Mapped");
        telemetry.update();

        //Reversing any motors that need to be reversed

        //Set zero power behavior
        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // ArmControlThread armControlThread = new ArmControlThread(robot.shoulderMotor, robot.elbowServoLeft, robot.elbowServoRight, robot.clamp, robot.wrist, robot.sensorFront, 10,10);

        ElapsedTime time = new ElapsedTime();

        Telemetry telemetry = new Telemetry(this,robot,10, false);

        //filewriterThread fileWriter = new filewriterThread(time, this.getClass().getSimpleName());

        elevatorThread elevatorAutonThread = new elevatorThread(robot.liftMotor, robot.tensionMotor, robot.rulerMotor, robot.clampL, robot.clampR, robot.capstone, 1, 0, 30000, 30000, 20, 20, robot.sensorFront,null);

        //RobotMovements.Locations skystonePosition = detectUsingBlueJay(false);

        drivingThread simpleDriveThread = new drivingThread(this,hardwareMap,robot, robot.sensorFront,robot.FL, robot.FR, robot.BL, robot.BR,10,3.0,1.0+1.0/8.0,null,elevatorAutonThread, telemetry, false,true,true,true,true,null);

        waitForStart();

        /**
         * Creates and starts the drive, elevator, and telemetry threads
         */

        Thread a = new Thread(simpleDriveThread);
        Thread b = new Thread(elevatorAutonThread);
        Thread c = new Thread(telemetry);
        //Thread d = new Thread(fileWriter);

        a.start();
        b.start();
        c.start();
        //d.start();



        while (!isStopRequested())
        {

        }
        /**
         * Stops the active threads and interrupts them so they don't keep running
         */
        if(isStopRequested())
        {
            simpleDriveThread.doStop();
            elevatorAutonThread.doStop();
            telemetry.doStop();
            //fileWriter.doStop();

            a.interrupt();
            b.interrupt();
            c.interrupt();
            //d.interrupt();

            requestOpModeStop();
            stop();
        }
        simpleDriveThread.doStop();
        elevatorAutonThread.doStop();
        telemetry.doStop();
        //fileWriter.doStop();

        a.interrupt();
        b.interrupt();
        c.interrupt();
        //d.interrupt();
    }


}
