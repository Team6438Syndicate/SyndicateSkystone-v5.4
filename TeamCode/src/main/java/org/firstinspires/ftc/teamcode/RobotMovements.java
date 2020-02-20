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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.jetbrains.annotations.NotNull;

public abstract class RobotMovements extends LinearOpMode
{
    protected Team6438ChassisHardwareMapCurrent robot = new Team6438ChassisHardwareMapCurrent();


    /**
     * Two Servo movements
     *
     * @param A1        Button 1
     * @param B1        Button 2
     * @param position1 Position for Button 1
     * @param position2 Position for Button 2
     * @param servo1    One servo
     * @param servo2    Another servo
     */
    protected void servoMovement(boolean A1, boolean B1, double position1, double position2, Servo servo1, Servo servo2) {

        if (A1) {
            servo1.setPosition(position1);
            servo2.setPosition(position1);

        } else if (B1) {
            servo1.setPosition(position2);
            servo2.setPosition(position2);

        }
    }

    /**
     * @param hardwareMap map of the robot devices
     */
    protected void initRobot(HardwareMap hardwareMap) {
        robot.init(hardwareMap);
    }

    /**
     * One Servo movements
     *
     * @param A1        Button 1
     * @param B1        Button 2
     * @param position1 Position for Button 1
     * @param position2 Position for Button 2
     * @param servo1    One servo
     */
    protected void servoMovement(boolean A1, boolean B1, double position1, double position2, Servo servo1) {

        if (A1) {
            servo1.setPosition(position1);

        } else if (B1) {
            servo1.setPosition(position2);

        }
    }

    /**
     * Moving the intake
     *
     * @param direction direction of the wheels spinning
     */
    protected void intakeMove(@NotNull directions direction) {
        switch (direction) {
            case intake:
                motorSetPower(- 1, robot.intakeLeft);
                motorSetPower(- 1, robot.intakeRight);

            case output:
                motorSetPower(1, robot.intakeLeft);
                motorSetPower(1, robot.intakeRight);

        }
    }

    /**
     * Normal operation of spinning motors
     *
     * @param power   power level of the motors
     * @param dcMotor the direct current motor to be controlled
     */
    protected void motorSetPower(double power, @NotNull DcMotor dcMotor) {
        dcMotor.setPower(power);
    }

    /**
     * Human control of the intake
     *
     * @param X1 one button for one direction
     * @param Y1 one button for another direction
     */
    protected void hControlIntakeMove(boolean X1, boolean Y1) {
        if (X1)// put out
        {
            intakeMove(directions.output);
        } else if (Y1) //take in
        {
            intakeMove(directions.intake);
        }
    }

    /**
     * Encoder driven motors
     *
     * @param position position to go to
     * @param power    power level of the motors
     * @param motor    the direct current motor to be controlled
     */
    protected void motorMovementEncoder(int position, double power, @NotNull DcMotor motor) {

        int currentPosition = motor.getCurrentPosition();

        motor.setTargetPosition(currentPosition + position);
        motor.setPower(power);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }





    private enum conditionCode {coord, angle}

    private enum coordCondition {returnFirst, returnSecond, returnAverage, rerunError}

    protected enum directions {intake, output}


}
