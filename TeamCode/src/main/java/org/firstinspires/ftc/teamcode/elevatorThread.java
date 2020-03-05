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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.odometry.Telemetry;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

//import com.qualcomm.robotcore.hardware.DistanceSensor;

public class elevatorThread implements Runnable
{
    //Variables to use
    private boolean userControlable;
    private double height = 0;
    private boolean halfSpeed = false;
    private int towerCount = 0;
    private long mills;
    private boolean doStop = false;
    private double capstorePos = .845;
    private float range;
    private int LIFT_MAX_VALUE;
    private int LIFT_MULTIPLIER_UP;
    private int LIFT_MULTIPLIER_DOWN;

    //References to threads
    private filewriterThread fileWriter;
    private Telemetry telemetry;

    //References to hardware
    private final DcMotor lift;
    private final DcMotor tension;
    private final DcMotor rulerMotor;
    private Servo lclamp;
    private Servo rclamp;
    private Servo foundationL;
    private Servo foundationR;
    private Servo capstone;
    private  Rev2mDistanceSensor frontSensor;

    //References to hardware map
    private HardwareMap hardwareMap;
    private Team6438ChassisHardwareMapCurrent robot = null;

    //Reference to gamepad
    private Gamepad gamepad;



    //___________________________________________________________________
    //Encoder Variables
    private  final double elevatorMotorCPR = 753.2;   //This is set for the 223
    //Drive Gear Reduction
    private  final double DGRLift = 1;
    //Wheel Diameter Mills
    private  final double WDMMLift = 31.9532; // TODO: 12/22/2019 Pulley size
    //Counts per Mills
    private final double CPMMLift = (elevatorMotorCPR)
                      / (WDMMLift / DGRLift * FastMath.PI) ;
    //Counts per in for slides
    public  final double CPILift =   CPMMLift / mmPerInch;

    //Counts per shaft revolution of the ruler motor
    public final double rulerMotorCPR = 145.6;

    //Wheel diam ruler
    public final double rulerWDI = 2;

    //Drive gear reduction for the ruler
    public final double DGRRuler = 1.0; //directDrive

    //Counts per inch for the ruler
    private final double rulerCPIInches = rulerMotorCPR / (rulerWDI/ DGRRuler * FastMath.PI);

    /**
     * constructor for the thread
     * @param lift
     * @param tension
     * @param rulerMotor
     * @param clampL
     * @param clampR
     * @param mills
     * @param gamepad
     * @param range
     * @param lift_multiplier_up
     * @param lift_multiplier_down
     * @param lift_max_value
     * @param tension_max_value
     * @param telemetry
     */
    public elevatorThread(@NotNull DcMotor lift, DcMotor tension, DcMotor rulerMotor, Servo clampL, Servo clampR, Servo foundationL, Servo foundationR, Servo capstone, final long mills, Gamepad gamepad, final float range, final int lift_multiplier_up, final int lift_multiplier_down, final int lift_max_value, final int tension_max_value, @NotNull Telemetry telemetry)
    {

        this.lift = lift;
        this.tension = tension;
        this.rulerMotor = rulerMotor;
        this.lclamp = clampL;
        this.rclamp = clampR;
        this.foundationL = foundationL;
        this.foundationR = foundationR;
        this.capstone = capstone;
        this.mills = mills;
        this.gamepad = gamepad;
        this.range = range;
        LIFT_MAX_VALUE = lift_max_value;
        LIFT_MULTIPLIER_UP = lift_multiplier_up;
        LIFT_MULTIPLIER_DOWN = lift_multiplier_down;

        this.telemetry = telemetry;
        //TENSION_MAX_VALUE = tension_max_value;

        userControlable = true;
        /*moveToZero();
        while(lift.isBusy() )
        {

        }*/
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rulerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rulerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // tension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //tension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        capstone.setPosition(capstorePos);

        //closeClamp();
    }

    public elevatorThread(@NotNull DcMotor lift, DcMotor tension, DcMotor rulerMotor, Servo lclamp, Servo rclamp, Servo capstone, final long mills, double height, final int lift_max_value, final int tension_max_value, final int lift_multiplier_up, final int lift_multiplier_down, Rev2mDistanceSensor Rev2mDistanceSensor, filewriterThread fileWriter)
    {
        userControlable = false;
        this.lift = lift;
        this.tension = tension;
        this.rulerMotor = rulerMotor;
        this.lclamp = lclamp;
        this.rclamp = rclamp;
        this.capstone = capstone;
        this.mills = mills;

        frontSensor= Rev2mDistanceSensor;
        this.height = height;
        LIFT_MAX_VALUE = lift_max_value;

        this.fileWriter = fileWriter;

        LIFT_MULTIPLIER_UP = lift_multiplier_up;
        LIFT_MULTIPLIER_DOWN = lift_multiplier_down;

        //TENSION_MAX_VALUE = tension_max_value;
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // tension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //tension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //capstone.setPosition(capstorePos);
    }

    /**
     * Stops the thread
     */
    public synchronized void doStop()
    {
        if (!userControlable)
        {
            //fileWriter.write("Elevator Thread Stopped");
        }

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
    public void run()
    {
        if(!userControlable)
        {
            startLift();
        }
        while (keepRunning())
        {
            try
            {
                if(userControlable)
                {
                    Thread.sleep(mills);

                    rulerMotor.setPower(-gamepad.left_stick_y);

                    if(!gamepad.a)
                    {
                        //close clamp
                        closeClamp();
                    }
                    else
                    {
                        //open clamps
                        openClamp();
                    }

                    if(gamepad.dpad_up)
                    {
                        towerCount++;
                        Thread.sleep(500);
                    }
                    else if (gamepad.dpad_down)
                    {
                        towerCount--;
                        if (towerCount < 0)
                        {
                            towerCount = 0;
                        }
                        Thread.sleep(300);
                    }

                    else if (gamepad.left_stick_button)
                    {
                        towerCount = 0;
                    }

                    if (gamepad.y && robot.foundationR.getPosition() == .9)
                    {
                        grabFoundation();
                    }
                    else if (gamepad.y && robot.foundationR.getPosition() != .9)
                    {
                        midFoundation();
                    }
                    else if (gamepad.x)
                    {
                        releaseFoundation();
                    }

                    if (gamepad.right_stick_button)
                    {
                        dropCapstone(0.375, 750);
                    }

                    if(gamepad.right_bumper)
                    {
                        moveToZero();
                        while(lift.isBusy() )
                        {

                        }
                    }

                    halfSpeed = gamepad.b;
                    MovementDistance tempStorage = resolveUserControl();
                    move(tempStorage);

                    checkBounds();

                    telemetry.append(Integer.toBinaryString(tempStorage.ticksForLift));
                    telemetry.append(Integer.toString(tempStorage.ticksForLift));
                    telemetry.update();

                }
                else
                {

                }
            }
            catch (InterruptedException ignored)
            {

            }
        }
    }

    //Start the lift to get the grippers out
    private void startLift()
    {
        move(resolveAutonMovement(15, 0));

        closeClamp();
        try
        {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        move(resolveAutonMovement(-15, 0));

        //Reset the encoder to prevent anomalies elsewhere
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    void move(MovementDistance movementDistance)
    {
        if(movementDistance.getTicksForLift() == 0 && movementDistance.getTicksForTension() == 0)
        {
            return;
        }
        int liftHeight = movementDistance.getTicksForLift();
        // int tensionRotation = movementDistance.getTicksForTension();

        liftHeight = lift.getTargetPosition() + liftHeight;
        //tensionRotation = tension.getCurrentPosition() + tensionRotation;


        lift.setTargetPosition(liftHeight);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(userControlable)
        {
            lift.setPower(1);
        }
        else
        {
            lift.setPower(1);
        }
    }

    /**
     * Code for controlling the movement of the elevator
     * @return
     */
    private MovementDistance resolveUserControl()
    {


        MovementDistance movementDistance = new MovementDistance(0, 0);
        if(gamepad.left_trigger > 0.0 + range && gamepad.right_trigger < 0.0 + range)
        {
            double inchDistance =  gamepad.left_trigger * LIFT_MULTIPLIER_UP;
            movementDistance.setTicksForLift(convertDistanceToTicks(inchDistance,MotorType.lift));
            movementDistance.setTicksForTension(convertDistanceToTicks(inchDistance,MotorType.tension));
        }
        else if (gamepad.left_trigger < 0.0 + range && gamepad.right_trigger > 0.0 + range)
        {
            double inchDistance =  gamepad.right_trigger * LIFT_MULTIPLIER_DOWN;
            movementDistance.setTicksForLift(-convertDistanceToTicks(inchDistance,MotorType.lift));
            movementDistance.setTicksForTension(-convertDistanceToTicks(inchDistance,MotorType.tension));
        }



        return movementDistance;
    }

    //Simplified version of resolveUserControl for autonomous use
    MovementDistance resolveAutonMovement(int liftDistance, int tensionDistance)
    {
        return new MovementDistance(convertDistanceToTicks(liftDistance, MotorType.lift), 0);
    }

    public void elevatorStart()
    {
        move(resolveAutonMovement(-2500, 0));
        while (FastMath.abs(lift.getCurrentPosition() + 2500) > 50)
        {

        }
        openClamp();
        while (FastMath.abs(lift.getCurrentPosition()) > 50)
        {

        }
        move(resolveAutonMovement(2500, 0));
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        move(resolveAutonMovement(0, 0));
    }


    //sets the elevator to the base position
    private void moveToZero()
    {
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // TODO: 1/2/2020 Check if this distance exceeds the minimum or maximum values
    private void checkBounds()
    {
        if(lift.getTargetPosition() < 0)
        {
            lift.setTargetPosition(0);
        }

        if(lift.getTargetPosition() > LIFT_MAX_VALUE)
        {
            lift.setTargetPosition(LIFT_MAX_VALUE);
        }


    }

    //converts inches to ticks

    private int convertDistanceToTicks(double distance, MotorType motorType)
    {

        try
        {
            switch (motorType)
            {
                case lift:
                    return (int) FastMath.round(distance * CPILift);
                case tension:
                    return 0;
                default:
                    throw new IllegalStateException("Unexpected value: " + motorType);
            }
        }


        catch (IllegalStateException e)
        {
            e.printStackTrace();
            return 0;
        }


    }

    //converts ticks to inches
    private double convertTicksToDistance(int ticks, MotorType motorType)
    {

        try
        {
            switch (motorType)
            {
                case lift:
                    return (double)lift.getCurrentPosition() / CPILift;
                case tension:
                    //return (int) FastMath.round(distance * hexCPITension);
                    return 0;
                default:
                    throw new IllegalStateException("Unexpected value: " + motorType);
            }
        }
        catch (IllegalStateException e)
        {
            e.printStackTrace();
            return 0;
        }


    }
    private enum MotorType
    {
        lift,tension
    }

    private static class MovementDistance
    {
        private int ticksForLift;
        private int ticksForTension;

        private MovementDistance(final int ticksForLift, final int ticksForTension)
        {
            this.ticksForLift = ticksForLift;
            this.ticksForTension = ticksForTension;
        }

        private int getTicksForLift()
        {
            return ticksForLift;
        }

        private void setTicksForLift(final int ticksForLift)
        {
            this.ticksForLift = ticksForLift;
        }

        private int getTicksForTension()
        {
            return ticksForTension;
        }

        private void setTicksForTension(final int ticksForTension)
        {
            this.ticksForTension = ticksForTension;
        }
    }

    void rulerPark()
    {
        rulerMotor.setTargetPosition(41*(int)rulerCPIInches);
        rulerMotor.setPower(1);
    }

    synchronized void openClamp()
    {
        lclamp.setPosition(.45);
        rclamp.setPosition(.55);
    }

    synchronized void closeClamp()
    {
        lclamp.setPosition(0.25);
        rclamp.setPosition(0.75);
    }

    synchronized void closeClampWide()
    {
        lclamp.setPosition(.6);
        rclamp.setPosition(.4);
    }

    void grabFoundation()
    {
        foundationL.setPosition(1);
        foundationR.setPosition(0);
    }

    void releaseFoundation()
    {
        foundationL.setPosition(.5);
        foundationR.setPosition(.5);
    }

    void midFoundation()
    {
        foundationL.setPosition(.9);
        foundationR.setPosition(.1);
    }

    void dropCapstone(double dropPosition, int time)
    {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double elapsedTime = timer.milliseconds();

        while (elapsedTime < time)
        {
            elapsedTime = timer.milliseconds();
            capstone.setPosition(dropPosition * elapsedTime/time);
        }

        while (elapsedTime < time + 50)
        {
            elapsedTime = timer.milliseconds();
        }
        capstone.setPosition(capstorePos);
    }
}
