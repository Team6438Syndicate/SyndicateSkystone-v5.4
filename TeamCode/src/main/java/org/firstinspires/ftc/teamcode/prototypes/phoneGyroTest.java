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

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.RobotMovements;
import org.firstinspires.ftc.teamcode.odometry.Telemetry;

import java.util.Objects;

@Autonomous(name = "PhoneGyroTest", group = "Dev")

public class phoneGyroTest extends RobotMovements {
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
        telemetry.update();

        Telemetry telemetry = new Telemetry(this,robot,10,false);
        ImuCorrections imuCorrections = new ImuCorrections(Objects.requireNonNull(AppUtil.getInstance().getActivity()).getApplicationContext(), telemetry);


        Thread a = new Thread(telemetry);
        Thread b = new Thread(imuCorrections);

        waitForStart();


        a.start();
        b.start();



        while (!isStopRequested())
        {

        }
        telemetry.doStop();
        imuCorrections.doStop();


        a.interrupt();
        b.interrupt();
    }
}
