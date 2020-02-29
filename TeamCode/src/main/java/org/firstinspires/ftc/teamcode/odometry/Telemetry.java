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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Team6438ChassisHardwareMapCurrent;
import org.jetbrains.annotations.Contract;

public class Telemetry implements Runnable
{
    private Team6438ChassisHardwareMapCurrent robot;
    private org.firstinspires.ftc.robotcore.external.Telemetry telemetry;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private int mills;
    private boolean teleOp = false;

    public Telemetry(OpMode opMode, Team6438ChassisHardwareMapCurrent robot, int mills, boolean teleOp)
    {

        this.robot = robot;
        this.telemetry = opMode.telemetry;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;
        this.mills = mills;
    }

    private boolean doStop = false;

    /**
     * Stops the thread
     */
   public synchronized void doStop() {
        this.doStop = true;
    }

    /**
     * Synchronized method to check if the tread should run
     * @return returns true to keep running if doStop equals false else returns false.
     */
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
        while (keepRunning())
        {
            try{
                Thread.sleep(mills);
            } catch (InterruptedException ignored) {
            }

            if (teleOp)
            {
                printTelem();
            }
        }
        telemetry.clearAll();
    }

    private void printTelem()
    {
        telemetry.addData("Front Distance: ", robot.sensorFront.getDistance(DistanceUnit.MM)-19);

        telemetry.addData("G1: left stick Y", gamepad1.left_stick_y);
        telemetry.addData("G1: left stick X", gamepad1.left_stick_x);
        telemetry.addData("G1: right stick Y", gamepad1.right_stick_y);
        telemetry.addData("G1: right stick X", gamepad1.right_stick_x);

        telemetry.addData("motor1", robot.FL.getPower());
        telemetry.addData("motor2", robot.BL.getPower());
        telemetry.addData("motor3", robot.FR.getPower());
        telemetry.addData("motor4", robot.BR.getPower());
        telemetry.addData("lift", robot.liftMotor.getPower());

        telemetry.addData("motor1CP", robot.FL.getCurrentPosition());
        telemetry.addData("motor2CP", robot.BL.getCurrentPosition());
        telemetry.addData("motor3CP", robot.FR.getCurrentPosition());
        telemetry.addData("motor4CP", robot.BR.getCurrentPosition());
        telemetry.addData("lift", robot.liftMotor.getCurrentPosition());

        telemetry.addData("motor1TP", robot.FL.getTargetPosition());
        telemetry.addData("motor2TP", robot.BL.getTargetPosition());
        telemetry.addData("motor3TP", robot.FR.getTargetPosition());
        telemetry.addData("motor4TP", robot.BR.getTargetPosition());
        telemetry.addData("lift", robot.liftMotor.getTargetPosition());

        telemetry.addData("G2: left stick Y", gamepad2.left_stick_y);
        telemetry.addData("G2: left stick X", gamepad2.left_stick_x);
        telemetry.addData("G2: right stick Y", gamepad2.right_stick_y);
        telemetry.addData("G2: right stick X", gamepad2.right_stick_x);
        int x =  (int) FastMath.round(Range.scale(gamepad2.left_stick_y,0.0,1.0,0,50));
        int y =  (int) FastMath.round(Range.scale(gamepad2.left_stick_y,0.0,1.0,0,20));

        telemetry.addData("G2: Scaled Value,X", x);
        telemetry.addData("G2: Scaled Value,Y", y);

        telemetry.update();
    }

    public void printTargetIndex(int targetIndex)
    {
        telemetry.addData("Target Index: ", targetIndex);
        telemetry.update();
    }

    public void append(String statement)
    {
        telemetry.addData("", statement);
    }

    public void update()
    {
        telemetry.update();
    }

    public void print(String statement)
    {
        telemetry.addData("", statement);
        telemetry.update();
    }

    public void speak(String statement)
    {
        telemetry.speak(statement);
    }

    public void printSpeeds(double pwr1, double pwr2, double pwr3, double pwr4)
    {
        telemetry.addData("Power FL: ", pwr1);
        telemetry.addData("Power FR: ", pwr2);
        telemetry.addData("Power BL: ", pwr3);
        telemetry.addData("Power BR: ", pwr4);
        telemetry.update();
    }

    public void printTargets(int tar1, int tar2, int tar3, int tar4)
    {
        telemetry.addData("Target FL: ", tar1);
        telemetry.addData("Target FR: ", tar2);
        telemetry.addData("Target BL: ", tar3);
        telemetry.addData("Target BR: ", tar4);
        telemetry.update();
    }
}
