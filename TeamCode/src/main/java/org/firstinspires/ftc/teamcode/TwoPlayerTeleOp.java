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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.odometry.Telemetry;

@TeleOp(name = "2 Player TeleOp", group = "!!!Team 6438 Driver Controlled")
public class TwoPlayerTeleOp extends RobotMovements {
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {

        try
        {
            //init the hardware
            initRobot(hardwareMap);
            //Telemetry
            telemetry.addData("Hardware Status:", "Mapped");
            telemetry.speak("Hardware Status: Ready To Go");
            telemetry.update();

            //Reversing any motors that need to be reversed


            //Set zero power behavior
            robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //Set motor mode
            robot.FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



            //Invoke the driving thread on controller 1(Kevin)
            drivingThread simpleDriveThread = new drivingThread(robot.FL, robot.FR, robot.BL, robot.BR, 1, gamepad1, 5.0, 1.0 + 1.0/4.0);

            //Invoke the telem thread
            Telemetry telemetry = new Telemetry(this,robot,10, true);

            //Invoke the elevator thread on controller 2(mark)
            elevatorThread elevatorThread = new elevatorThread(robot.liftMotor, robot.tensionMotor, robot.rulerMotor, robot.clampL, robot.clampR, robot.foundationL, robot.foundationR, robot.capstone, 1, gamepad2, .1f, 50, 20, 300000, 300000, telemetry);

            //Form the threads
            Thread a = new Thread(simpleDriveThread);
            Thread b = new Thread(telemetry);
            Thread c = new Thread(elevatorThread);

            //Wait for player to press start
            waitForStart();

            //Start the threads
            a.start();
            b.start();
            c.start();

            //While opmode
            while (!isStopRequested())
            {

            }

            //After stop is requested
            simpleDriveThread.doStop();
            elevatorThread.doStop();
            telemetry.doStop();

            //Intterupt the threads
            a.interrupt();
            b.interrupt();
            c.interrupt();
            stop();
            requestOpModeStop();
            this.telemetry.clear();
        }
        catch (Exception e)
        {
            telemetry.addData("Error", e.getStackTrace().toString());
        }
    }



}
