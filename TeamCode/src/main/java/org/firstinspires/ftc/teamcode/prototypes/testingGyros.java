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

package org.firstinspires.ftc.teamcode.prototypes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotMovements;
import org.firstinspires.ftc.teamcode.odometry.PositionFileWriteOutThread;
import org.firstinspires.ftc.teamcode.odometry.Telemetry;

@Autonomous(name = "IMUGyroTurn", group = "Dev")
public class testingGyros extends RobotMovements {

    public testingGyros()
    {

    }



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
        initRobot(hardwareMap);
        //Telemetry
        telemetry.addData("Hardware Status:", "Mapped");
        telemetry.addData("IMU: " , robot.imu.toString());
        telemetry.addData("Angles: " , robot.imu.getAngularOrientation());

        telemetry.update();

        //Reversing any motors that need to be reversed


        //Set zero power behavior
        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        robot.FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Telemetry telemetry = new Telemetry(this,robot,10,false);

        ImuCorrections imuCorrections = new ImuCorrections(robot,telemetry,robot.FL,robot.FR,robot.BL,robot.BR, .03, false);

        PositionFileWriteOutThread positionFileWriteOutThread = new PositionFileWriteOutThread(this,robot,false);

        Thread a = new Thread(telemetry);

        Thread b = new Thread(imuCorrections);

        Thread c = new Thread(positionFileWriteOutThread);

        waitForStart();


        a.start();

        b.start();

        c.start();


        while (!isStopRequested())
        {

        }


        telemetry.doStop();

        imuCorrections.doStop();

        positionFileWriteOutThread.doStop();

        a.interrupt();
        b.interrupt();
        c.interrupt();
    }
}

