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

@TeleOp(name = "Run this OpMode TeleOp", group = "Team 6438 Driver Controlled")
public class Team6438ThreadedTeleOp extends RobotMovements {
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
            telemetry.update();

            //Reversing any motors that need to be reversed


            //Set zero power behavior
            robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            //robot.intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //robot.intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


            //robot.shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            robot.FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            //robot.intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //robot.intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //robot.shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            drivingThread simpleDriveThread = new drivingThread(robot.FL, robot.FR, robot.BL, robot.BR, 1, gamepad1, 5.0, 1.0 + 1.0/4.0);

            Telemetry telemetry = new Telemetry(this,robot,10, true);

            elevatorThread elevatorThread = new elevatorThread(robot.liftMotor, robot.tensionMotor, robot.clampL, robot.clampR, robot.foundationL, robot.foundationR, 1, gamepad1, .1f, 50, 20, 300000, 300000, telemetry);

            Thread a = new Thread(simpleDriveThread);

            Thread b = new Thread(telemetry);

            Thread c = new Thread(elevatorThread);


            waitForStart();

            a.start();
            b.start();

            c.start();


            while (!isStopRequested())
            {

            }

            simpleDriveThread.doStop();
            elevatorThread.doStop();
            telemetry.doStop();

            a.interrupt();
            b.interrupt();
            c.interrupt();


            this.telemetry.clear();
        }
        catch (Exception e)
        {
            telemetry.addData("Error", e.getStackTrace().toString());
        }
    }



}
