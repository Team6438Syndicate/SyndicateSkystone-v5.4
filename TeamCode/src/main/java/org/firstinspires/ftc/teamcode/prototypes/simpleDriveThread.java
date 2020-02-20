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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Team6438ChassisHardwareMapCurrent;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

@Deprecated
@Disabled
public class simpleDriveThread implements Runnable {
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private int mills;
    private Gamepad gamepad;
    private double distance;
    private double factorSpeedDown;
    private double factorSpeedUp;
    private boolean userControllable = false;
    private Team6438ChassisHardwareMapCurrent robot = null;
    private boolean doStop = false;

    simpleDriveThread(@NotNull DcMotor motor1, @NotNull DcMotor motor2, @NotNull DcMotor motor3, @NotNull DcMotor motor4, int mills, Gamepad gamepad, double scaleUp, double scaleDown) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.motor3 = motor3;
        this.motor4 = motor4;
        this.mills = mills;
        this.factorSpeedDown = scaleDown;
        this.factorSpeedUp = scaleUp;
        userControllable = true;
        this.gamepad = gamepad;
    }

    synchronized void doStop() {
        this.doStop = true;
    }

    @Contract(pure = true)
    private synchronized boolean keepRunning() {
        return ! this.doStop;
    }

    /**
     * When an object implementing interface <code>Runnable</code> is used
     * to create a thread, starting the thread causes the object's
     * <code>run</code> method to be called in that separately executing
     * thread.
     * <p>
     * The general contract of the method <code>run</code> is that it may
     * take any action whatsoever.
     *
     * @see Thread#run()
     */
    @Override
    public void run() {
        while (keepRunning()) {
            motor1.setPower(- gamepad.left_stick_y);
            motor2.setPower(- gamepad.left_stick_y);
            motor3.setPower(- gamepad.right_stick_y);
            motor4.setPower(- gamepad.right_stick_y);

            try {
                Thread.sleep(mills);

            } catch (InterruptedException ignored) {

            }

        }
    }
}