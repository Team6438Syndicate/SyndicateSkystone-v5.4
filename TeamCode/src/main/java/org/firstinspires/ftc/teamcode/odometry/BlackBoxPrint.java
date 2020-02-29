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

package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "Print Position")
public class BlackBoxPrint extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private BlackBoxBot bb = new BlackBoxBot();

    public void runOpMode() throws InterruptedException {
        bb.init(hardwareMap);
        runtime.reset();
        bb.resetTicks();
        telemetry.addData("status", "initialized");
        telemetry.update();
        waitForStart();
        while (! isStopRequested() && opModeIsActive()) {
            bb.updatePosition();
            telemetry.addData("Left ticks", bb.getLeftTicks());
            telemetry.addData("Center ticks", bb.getCenterTicks());
            telemetry.addData("Right ticks", bb.getRightTicks());
            telemetry.addData("X value", bb.getX());
            telemetry.addData("Y value", bb.getY());
            telemetry.addData("Theta value", bb.getTheta());
            telemetry.update();
            idle();
        }
    }
}
