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
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.Team6438ChassisHardwareMapCurrent;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

@Deprecated
@Disabled
public class threadDrive implements Runnable {
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
    threadDrive(@NotNull DcMotor motor1, @NotNull DcMotor motor2, @NotNull DcMotor motor3, @NotNull DcMotor motor4, int mills, Gamepad gamepad, double scaleUp, double scaleDown) {
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
    threadDrive(@NotNull Team6438ChassisHardwareMapCurrent robot, @NotNull DcMotor motor1, @NotNull DcMotor motor2, @NotNull DcMotor motor3, @NotNull DcMotor motor4, int mills, double distance, double scaleUp, double scaleDown) {
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
        float oldStickValS1;
        float currentLeftStickYVal = 0;

        float oldStickValS2;
        float currentRightStickYVal = 0;

        while (keepRunning()) {
            if (userControllable) {
                oldStickValS1 = currentLeftStickYVal;
                oldStickValS2 = currentRightStickYVal;

                currentLeftStickYVal = - gamepad.left_stick_y;
                currentRightStickYVal = - gamepad.right_stick_y;

                try {
                    Thread.sleep(mills);
                } catch (InterruptedException ignored) {

                }

                double range = .01;
                if (currentLeftStickYVal > oldStickValS1 && currentRightStickYVal > oldStickValS2) {
                    accelerate(motor1, currentLeftStickYVal);
                    accelerate(motor2, currentLeftStickYVal);
                    accelerate(motor3, currentRightStickYVal);
                    accelerate(motor4, currentRightStickYVal);
                    continue; //both bank control override

                } else if (currentLeftStickYVal < oldStickValS1 && currentRightStickYVal < oldStickValS2) {
                    decelerate(motor1, currentLeftStickYVal);
                    decelerate(motor2, currentLeftStickYVal);
                    decelerate(motor3, currentRightStickYVal);
                    decelerate(motor4, currentRightStickYVal);
                    continue; //both bank control override

                } else if (currentLeftStickYVal > oldStickValS1 && currentRightStickYVal < oldStickValS2) {
                    accelerate(motor1, currentLeftStickYVal);
                    accelerate(motor2, currentLeftStickYVal);
                    decelerate(motor3, currentRightStickYVal);
                    decelerate(motor4, currentRightStickYVal);
                    continue; //both bank control override

                } else if (currentLeftStickYVal < oldStickValS1 && currentRightStickYVal > oldStickValS2) {
                    decelerate(motor1, currentLeftStickYVal);
                    decelerate(motor2, currentLeftStickYVal);
                    accelerate(motor3, currentRightStickYVal);
                    accelerate(motor4, currentRightStickYVal);
                    continue; //both bank control override

                } else if (FastMath.abs(currentLeftStickYVal) - FastMath.abs(oldStickValS1) < range && FastMath.abs(currentRightStickYVal) - FastMath.abs(oldStickValS2) < range) {
                    decelerate(motor1, 0);
                    decelerate(motor2, 0);
                    decelerate(motor3, 0);
                    decelerate(motor4, 0);
                    continue;
                } else if (FastMath.abs(currentLeftStickYVal) - FastMath.abs(oldStickValS1) < range) {
                    decelerate(motor1, 0);
                    decelerate(motor2, 0);

                    continue;
                } else if (FastMath.abs(currentRightStickYVal) - FastMath.abs(oldStickValS2) < range) {
                    decelerate(motor3, 0);
                    decelerate(motor4, 0);
                    continue;
                }

                if (currentLeftStickYVal > oldStickValS1) {
                    accelerate(motor1, currentLeftStickYVal);
                    accelerate(motor2, currentLeftStickYVal);

                } else if (currentLeftStickYVal < oldStickValS1) {
                    decelerate(motor1, currentLeftStickYVal);
                    decelerate(motor2, currentLeftStickYVal);

                }

                if (currentRightStickYVal > oldStickValS2) {
                    accelerate(motor3, currentRightStickYVal);
                    accelerate(motor4, currentRightStickYVal);

                } else if (currentRightStickYVal < oldStickValS2) {
                    decelerate(motor3, currentRightStickYVal);
                    decelerate(motor4, currentRightStickYVal);
                }
            } else {
               /* double currentVelo = FastMath.hypot(robot.imu.getVelocity().xVeloc, robot.imu.getVelocity().yVeloc);

                accelOverDistance(motor1, 1, currentVelo);
                accelOverDistance(motor2, 1, currentVelo);
                accelOverDistance(motor3, 1, currentVelo);
                accelOverDistance(motor4, 1, currentVelo);

                */

            }

        }
    }


    /**
     * @param motor
     * @param finalPower
     */
    private void decelerate(@NotNull DcMotor motor, double finalPower) {
        double powerDrop;

        double signPower = motor.getPower();

        double power1 = FastMath.abs(signPower);


        if (power1 > finalPower) {

            powerDrop = 0.1 * FastMath.pow(power1, factorSpeedDown) + 0.1;

            motor.setPower(signPower - FastMath.copySign(powerDrop, signPower));

        }

    }

    /**
     * @param motor
     * @param finalPower
     */
    private void accelerate(@NotNull DcMotor motor, double finalPower) {
        double powerDrop;

        double signPower = motor.getPower();

        double power1 = FastMath.abs(signPower);


        if (power1 < finalPower) {

            powerDrop = 0.1 * FastMath.pow(power1, factorSpeedUp) + 0.1;

            motor.setPower(signPower + FastMath.copySign(powerDrop, signPower));

        }

    }

    /**
     * @param motor
     * @param finalVelocity
     * @param currentVelo
     */
    private void decelOverDistance(DcMotor motor, double finalVelocity, double currentVelo) {
        if (currentVelo > FastMath.abs(finalVelocity)) {
            double power = Range.clip(finalVelocity, - 1.0, 1.0);

            decelerate(motor, power);
        }

    }

    /**
     * @param motor
     * @param finalVelocity
     * @param currentVelo
     */
    private void accelOverDistance(DcMotor motor, double finalVelocity, double currentVelo) {
        if (currentVelo < FastMath.abs(finalVelocity)) {
            double power = Range.clip(finalVelocity, - 1.0, 1.0);

            decelerate(motor, power);
        }
    }

    /**
     * @return
     */
    public double getDistance() {
        return distance;
    }

    /**
     * @param distance
     */
    public void setDistance(double distance) {
        this.distance = distance;
    }
}
