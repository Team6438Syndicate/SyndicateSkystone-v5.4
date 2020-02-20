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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.Team6438ChassisHardwareMapCurrent;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

@Deprecated
public class bradsMistake implements Runnable {
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

    /**
     * @param motor1
     * @param motor2
     * @param motor3
     * @param motor4
     * @param mills
     * @param gamepad
     * @param scaleUp
     * @param scaleDown
     */
    bradsMistake(@NotNull DcMotor motor1, @NotNull DcMotor motor2, @NotNull DcMotor motor3, @NotNull DcMotor motor4, int mills, Gamepad gamepad, double scaleUp, double scaleDown) {
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

    /**
     * @param robot
     * @param motor1
     * @param motor2
     * @param motor3
     * @param motor4
     * @param mills
     * @param distance
     * @param scaleUp
     * @param scaleDown
     */
    bradsMistake(@NotNull Team6438ChassisHardwareMapCurrent robot, @NotNull DcMotor motor1, @NotNull DcMotor motor2, @NotNull DcMotor motor3, @NotNull DcMotor motor4, int mills, double distance, double scaleUp, double scaleDown) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.motor3 = motor3;
        this.motor4 = motor4;
        this.mills = mills;
        this.factorSpeedDown = scaleDown;
        this.factorSpeedUp = scaleUp;
        this.distance = distance;
        this.robot = robot;
        //robot.imu.startAccelerationIntegration(new Position(), new Velocity(), mills);
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
        double gamepadLY;
        double currentSpeedLeft = 0;

        double gamepadRY;
        double currentSpeedRight = 0;

        double range = 0.02;

        while (keepRunning()) {
            currentSpeedLeft = motor1.getPower();
            currentSpeedRight = motor2.getPower();

            gamepadLY = - gamepad.left_stick_y;
            gamepadRY = - gamepad.right_stick_y;

            //Left Wheels
            if (gamepadLY >= 0) //for positive values
            {
                if (currentSpeedLeft < 0) {
                    decelerate(motor1, motor3, 0);
                } else if (gamepadLY - currentSpeedLeft > range) {
                    accelerate(motor1, motor3, gamepadLY);
                } else if (gamepadLY - currentSpeedLeft < - range) {
                    decelerate(motor1, motor3, gamepadLY);
                }
            } else //for negative values
            {
                if (currentSpeedLeft > 0) {
                    decelerate(motor1, motor3, 0);
                } else if (gamepadLY - currentSpeedLeft > range) {
                    decelerate(motor1, motor3, gamepadLY);
                } else if (gamepadLY - currentSpeedLeft < - range) {
                    accelerate(motor1, motor3, gamepadLY);
                }
            }

            //Right Wheels
            if (gamepadRY >= 0) //for positive values
            {
                if (currentSpeedRight < 0) {
                    decelerate(motor2, motor4, 0);
                } else if (gamepadRY - currentSpeedRight > range) {
                    accelerate(motor2, motor4, gamepadRY);
                } else if (gamepadRY - currentSpeedRight < - range) {
                    decelerate(motor2, motor4, gamepadRY);
                }
            } else //for negative values
            {
                if (currentSpeedRight > 0) {
                    decelerate(motor2, motor4, 0);
                } else if (gamepadRY - currentSpeedRight > range) {
                    decelerate(motor2, motor4, gamepadRY);
                } else if (gamepadRY - currentSpeedRight < - range) {
                    accelerate(motor2, motor4, gamepadRY);
                }
            }
            try {
                Thread.sleep(mills);
            } catch (InterruptedException ignored) {

            }
        }
    }


    /**
     * @param motorFront
     * @param motorBack
     * @param finalPower
     */
    private void decelerate(@NotNull DcMotor motorFront, DcMotor motorBack, double finalPower) {

        double powerDrop = 0;

        double signPower = motorFront.getPower();

        double power1 = FastMath.abs(signPower);

        if (power1 - powerDrop > FastMath.abs(finalPower)) {
            signPower = motorFront.getPower();

            power1 = FastMath.abs(signPower);

            powerDrop = 0.25 * FastMath.pow(power1, factorSpeedDown) + 0.01;

            power1 = signPower - FastMath.copySign(powerDrop, signPower);
            motorFront.setPower(power1);
            motorBack.setPower(power1);


        }

    }


    /**
     * @param motorFront
     * @param motorBack
     * @param finalPower
     */
    private void accelerate(@NotNull DcMotor motorFront, DcMotor motorBack, double finalPower) {
        double powerJump = 0;

        double signPower = motorFront.getPower();

        double power1 = FastMath.abs(signPower);

        while (power1 + powerJump < FastMath.abs(finalPower)) {
            signPower = motorFront.getPower();

            power1 = FastMath.abs(signPower);

            powerJump = 0.25 * FastMath.pow(power1, factorSpeedUp) + 0.01;


            power1 = signPower + FastMath.copySign(powerJump, signPower);
            motorFront.setPower(power1);
            motorBack.setPower(power1);

            try {
                Thread.sleep(mills);
            } catch (InterruptedException ignored) {

            }

        }
    }

}
