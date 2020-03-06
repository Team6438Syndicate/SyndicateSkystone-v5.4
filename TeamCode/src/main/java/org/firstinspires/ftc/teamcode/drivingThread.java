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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.pathfinder.PathFinder;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import detectors.FoundationPipeline.Pipeline;
import detectors.FoundationPipeline.SkyStone;
import detectors.OpenCvDetector;

import static org.apache.commons.math3.util.FastMath.PI;
import static org.apache.commons.math3.util.FastMath.abs;
import static org.apache.commons.math3.util.FastMath.round;
import static org.apache.commons.math3.util.FastMath.sin;
import static org.firstinspires.ftc.teamcode.Team6438ChassisHardwareMapCurrent.VUFORIA_KEY;
import static org.firstinspires.ftc.teamcode.Team6438ChassisHardwareMapCurrent.radiusMM;

//import com.qualcomm.robotcore.hardware.DistanceSensor;

/**
 * Thread for driving
 */
public class drivingThread implements Runnable {

    boolean objectDetected = false;
    double scanDistance = 0;
    OpenCvCamera webcam;
    private Rev2mDistanceSensor frontSensor;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private int mills;
    private Gamepad gamepad;
    private double factorSpeedDown;
    private double factorSpeedUp;
    private elevatorThread elevatorThread;
    private filewriterThread fileWriter;
    private org.firstinspires.ftc.teamcode.odometry.Telemetry telemetry;
    private boolean abortAfterFoundation;
    private boolean doubleSample;
    private boolean userControllable;
    private HardwareMap hardwareMap;
    private Team6438ChassisHardwareMapCurrent robot = null;
    private boolean doStop = false;
    public boolean runDetect = true;
    private int counter = 0;
    private boolean firstLoop = true;     //Holds the value of whether or not the foundation is in the spot where it should be
    private boolean isRed = false;
    private OpMode opmode;
    private boolean foundationMoveRequest;
    private boolean onlyFoundation;
    private boolean runOpenCV;
    private boolean stopScan = false;
    private DistanceUnit distanceUnit = DistanceUnit.INCH;
    private double oldHeadingIMU;
    private double newHeadingIMU;
    private double gain;
    private RobotMovements.Locations skystonePosition;
    private OpenCvDetector fieldElementDetector = null;
    private double distanceTo;
    private SkyStone[] skyStones;

    /**
     * Teleop control constructor
     *
     * @param motor1    first motor
     * @param motor2    second motor
     * @param motor3    third motor
     * @param motor4    fourth motor
     * @param mills     millisecond delay for the thread
     * @param gamepad   gamepad to control robot
     * @param scaleUp   speedup factor
     * @param scaleDown slowdown factor
     */
    drivingThread(@NotNull DcMotor motor1, @NotNull DcMotor motor2, @NotNull DcMotor motor3, @NotNull DcMotor motor4, Rev2mDistanceSensor sensorFront, int mills, Gamepad gamepad, double scaleUp, double scaleDown)
    {

        this.motor1 = motor1;
        this.motor2 = motor2;
        this.motor3 = motor3;
        this.motor4 = motor4;
        this.mills = mills;
        frontSensor = sensorFront;
        this.factorSpeedDown = scaleDown;
        this.factorSpeedUp = scaleUp;
        userControllable = true;
        this.gamepad = gamepad;
    }

    drivingThread(final OpMode opmode, final HardwareMap hardwareMap, @NotNull Team6438ChassisHardwareMapCurrent robot, Rev2mDistanceSensor sensorFront, @NotNull DcMotor motor1, @NotNull DcMotor motor2, @NotNull DcMotor motor3, @NotNull DcMotor motor4, int mills, double scaleUp, double scaleDown, filewriterThread fileWriter, elevatorThread elevatorThread, org.firstinspires.ftc.teamcode.odometry.Telemetry telemetry, boolean redCheck, boolean foundationMoveRequest, boolean abortAfterFoundation, boolean doubleSample, boolean runOpenCV, RobotMovements.Locations skystonePosition)
    {
        this.opmode = opmode;
        this.foundationMoveRequest = foundationMoveRequest;
        this.hardwareMap = hardwareMap;


        this.robot = robot;
        this.webcam = webcam;
        this.frontSensor = sensorFront;
        this.isRed = redCheck;
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.motor3 = motor3;
        this.motor4 = motor4;



        this.mills = mills;
        this.factorSpeedDown = scaleDown;
        this.factorSpeedUp = scaleUp;

        this.elevatorThread = elevatorThread;
        this.fileWriter = fileWriter;

        this.telemetry = telemetry;
        this.abortAfterFoundation = abortAfterFoundation;
        this.doubleSample = doubleSample;
        this.foundationMoveRequest = foundationMoveRequest;

        this.userControllable = false;

        this.runOpenCV = runOpenCV;

        if (foundationMoveRequest)
        {
            counter = 999;
        }

        //RobotMovements.initBlueJay();

        //createVuforia();

        if (!runOpenCV)
        {
            initVuforia();

            if (ClassFactory.getInstance().canCreateTFObjectDetector())
            {
                initTfod();

                if (robot.tfod != null)
                {
                    robot.tfod.setClippingMargins(10, 10, 10, 10);
                    //Activates the tfod engine
                    robot.tfod.activate();
                    robot.tfod.deactivate();
                    //Waits for .1 seconds to allow processor to catch up
                    try
                    {
                        Thread.sleep(10);
                    }
                    catch (Exception e)
                    {

                    }
                }
            }
        }
        else
        {
            //findSkystone(webcam);

            // TODO: 3/2/2020 integration in progress
            /*
            fieldElementDetector = new OpenCvDetector(telemetry.getOpmode(),true,hardwareMap);
            fieldElementDetector.start();
             */
        }
        fieldElementDetector = new OpenCvDetector(opmode,true,hardwareMap);
        fieldElementDetector.start();
        Pipeline.doSkyStones = true;
        Pipeline.doStones = false;
        Pipeline.doFoundations = false;

        this.robot.imu.startAccelerationIntegration(new Position(), new Velocity(), mills);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Power math is done from 0.0-1.0 then the sign is copied using Fastmath.copySign(double magnitude,double direction) in order to expand to -1.0-1.0
     *
     * @param motor       the given motor for th operation
     * @param powerSign   the direction on the stick or the def
     * @param directAccel the direction of acceleration
     * @param accelFactor the scale factor of the acceleration
     *
     * @return the relative speed change as a double
     */
    private static double speedChange(@NotNull DcMotor motor, double powerSign, DirectAccel directAccel, double accelFactor)
    {
        double oldPower = FastMath.abs(motor.getPower());

        if (oldPower < 0.01)
        {
            if (directAccel.equals(DirectAccel.accel))
            {
                oldPower = FastMath.copySign(0.01, powerSign);
            }
            else
            {
                return 0;
            }
        }

        oldPower = FastMath.abs(oldPower);
        double stepPower = FastMath.nextAfter(oldPower, accelType(directAccel)); //steps to the next machine printable number in the direction of directAccel

        stepPower -= oldPower;

        stepPower *= accelFactor * FastMath.pow(10, 15); //scales the number by a factor of accelFactor * 10^15

        double newPower = oldPower + stepPower;

        newPower = FastMath.copySign(newPower, powerSign);

        newPower = isZero(newPower);

        newPower = Range.clip(newPower, - 1.0, 1.0);

        // System.out.println(newPower);
        return newPower;
    }

    /**
     * @param directAccel which direction to increase or decrease the motors power.
     *
     * @return ∞:accel, -∞:decel
     */
    private static double accelType(DirectAccel directAccel)
    {
        if (directAccel.equals(DirectAccel.accel))
        {
            return Double.POSITIVE_INFINITY;
        }
        return Double.NEGATIVE_INFINITY;
    }

    /**
     * @param power Value evaluated to see if it is close to 0
     *
     * @return power level to set the motor to
     */
    private static double isZero(double power)
    {
        if (power < 0.01 && power > - 0.01)
        {
            return 0;
        }
        return power;
    }

    /**
     * Stops the thread
     */
    synchronized void doStop()
    {
        if (!userControllable)
        {
            fieldElementDetector.end();
        }

        this.doStop = true;
    }

    synchronized void stopScan()
    {
        if (!userControllable)
        {
        }

        this.stopScan = true;
    }

    /**
     * Synchronized method to check if the tread should run
     *
     * @return returns true to keep running if doStop equals false else returns false.
     */
    @Contract(pure = true)
    private synchronized boolean keepRunning()
    {
        return ! this.doStop;
    }

    @Contract(pure = true)
    private synchronized boolean keepScanning()
    {
        return ! this.stopScan;
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
        try
        {
            if (!userControllable)
            {
                while(keepScanning())
                {
                    skyStones = fieldElementDetector.getSkyStones();
                    telemetry.print("In scanning loop");

                    for (SkyStone detection : skyStones)
                    {
                        if (!keepScanning())
                        {
                            break;
                        }
                        if (detection != null)
                        {

                                if (!isRed)
                                {
                                    if (detection.y < fieldElementDetector.getWidth()/3.0)
                                    {
                                        this.skystonePosition = RobotMovements.Locations.Far;
                                        telemetry.print("Far");
                                    }
                                    else if (detection.y > fieldElementDetector.getWidth()*2/3.0)
                                    {
                                        this.skystonePosition = RobotMovements.Locations.Close;
                                        telemetry.print("Close");

                                    }
                                    else
                                    {
                                        this.skystonePosition = RobotMovements.Locations.Center;
                                        telemetry.print("Center");
                                    }
                                }
                                else
                                {
                                    telemetry.print("In else");
                                    if (detection.y < fieldElementDetector.getWidth()/3.0)
                                    {
                                        this.skystonePosition = RobotMovements.Locations.Close;
                                        telemetry.print("Close");

                                    }
                                    else if (detection.y > fieldElementDetector.getWidth()*2/3.0)
                                    {
                                        this.skystonePosition = RobotMovements.Locations.Far;
                                        telemetry.print("Far");
                                    }
                                    else
                                    {
                                        this.skystonePosition =  RobotMovements.Locations.Center;
                                        telemetry.print("Center");
                                    }
                                }


                        }

                       // telemetry.print(skystonePosition.toString());

                    }
                }
                telemetry.print(skystonePosition.toString(),"exited scanning loop");

                fieldElementDetector.end();
            }




            double currentSpeedLeft;

            double gamepadLY;

            double gamepadRX;

            double gamepadLX;

            final double MIN_Value = 0.01;

            double distanceFrontLeft = 0.0;
            double distanceBackLeft = 0.0;
            double distanceFrontRight = 0.0;
            double distanceBackRight = 0.0;
            ArrayList<ArrayList<Float>> old = new ArrayList<>();
            PathFinder.Path currentPath = null;
            double[] dis = null;


            int current = 0;

            while (keepRunning())
            {

                if (userControllable)
                {
                    double newMotorPower1 = 0.0;
                    double newMotorPower2 = 0.0;
                    double newMotorPower3 = 0.0;
                    double newMotorPower4 = 0.0;

                    try
                    {
                        Thread.sleep(mills);
                    }
                    catch (InterruptedException ignored)
                    {

                    }

                    if (gamepad.dpad_left)
                    {
                        motor1.setPower(- 0.55);
                        motor2.setPower(- 0.55);
                        motor3.setPower(0.55);
                        motor4.setPower(0.55);
                        continue;
                    }


                    if (gamepad.dpad_right)
                    {
                        motor1.setPower(0.5);
                        motor2.setPower(0.5);
                        motor3.setPower(- 0.5);
                        motor4.setPower(- 0.5);
                        continue;
                    }

                    if(gamepad.right_trigger>0.05)
                    {
                        motor1.setPower(.25 * gamepad.right_trigger);
                        motor2.setPower(-.25 * gamepad.right_trigger);
                        motor3.setPower(.25 * gamepad.right_trigger);
                        motor4.setPower(-.25 * gamepad.right_trigger);
                        continue;
                    }
                    if(gamepad.left_trigger>0.05)
                    {
                        motor1.setPower(-.25 * gamepad.left_trigger);
                        motor2.setPower(.25 * gamepad.left_trigger);
                        motor3.setPower(-.25 * gamepad.left_trigger);
                        motor4.setPower(.25 * gamepad.left_trigger);
                        continue;
                    }

                    if (gamepad.dpad_up)
                    {
                        motor1.setPower(0.5);
                        motor2.setPower(0.5);
                        motor3.setPower(0.5);
                        motor4.setPower(0.5);
                        continue;
                    }

                    if (gamepad.dpad_down)
                    {
                        motor1.setPower(- 0.5);
                        motor2.setPower(- 0.5);
                        motor3.setPower(- 0.5);
                        motor4.setPower(- 0.5);
                        continue;
                    }
                    if (gamepad.y)
                    {
                        motor1.setPower(0.25);
                        motor2.setPower(0.25);
                        motor3.setPower(0.25);
                        motor4.setPower(0.25);
                        continue;
                    }

                    if (gamepad.a)
                    {
                        motor1.setPower(- 0.25);
                        motor2.setPower(- 0.25);
                        motor3.setPower(- 0.25);
                        motor4.setPower(- 0.25);
                        continue;
                    }

                    if (gamepad.x)
                    {
                        //motor1.setTargetPosition((int) target);
                        motor1.setPower(- 0.25);
                        motor2.setPower(- 0.25);
                        motor3.setPower(0.25);
                        motor4.setPower(0.25);
                        continue;
                    }

                    if (gamepad.b)
                    {
                        motor1.setPower(0.25);
                        motor2.setPower(0.25);
                        motor3.setPower(- 0.25);
                        motor4.setPower(- 0.25);
                        continue;
                    }

                    if (gamepad.left_stick_button && gamepad.right_stick_button)
                    {
                        motor1.setPower(0);
                        motor2.setPower(0);
                        motor3.setPower(0);
                        motor4.setPower(0);
                        continue;
                    }





                    boolean accurateDrive = false;
                    if (gamepad.left_bumper)
                    {
                        accurateDrive = true;
                    }
                    else
                    {
                        accurateDrive = false;
                    }

                    if (accurateDrive)
                    {
                        factorSpeedUp /= 3;
                        factorSpeedDown /= 3;
                    }

                    currentSpeedLeft = motor1.getPower();

                    gamepadLY = - gamepad.left_stick_y;
                    gamepadRX = gamepad.right_stick_x;
                    gamepadLX = - gamepad.left_stick_x;


                    //Forward and Backwards
                    {
                        if (FastMath.abs(gamepadLY) < MIN_Value && FastMath.abs(gamepadLX) < MIN_Value && FastMath.abs(gamepadRX) < MIN_Value)
                        {
                            newMotorPower1 += speedChange(motor1, FastMath.copySign(0, motor1.getPower()), DirectAccel.decel, factorSpeedDown);
                            newMotorPower2 += speedChange(motor2, FastMath.copySign(0, motor2.getPower()), DirectAccel.decel, factorSpeedDown);
                            newMotorPower3 += speedChange(motor3, FastMath.copySign(0, motor3.getPower()), DirectAccel.decel, factorSpeedDown);
                            newMotorPower4 += speedChange(motor4, FastMath.copySign(0, motor4.getPower()), DirectAccel.decel, factorSpeedDown);
                        }

                        DirectAccel direction = DirectAccel.decel;
                        double factor = factorSpeedDown;
                        if (gamepadLY > 0) //for positive values

                        {

                            if (gamepadLY - currentSpeedLeft > MIN_Value)
                            {
                                factor = factorSpeedUp;
                                direction = DirectAccel.accel;
                            }


                        }
                        else if (gamepadLY < 0) //for negative values
                        {

                            if (gamepadLY - currentSpeedLeft < - MIN_Value)
                            {
                                factor = factorSpeedUp;
                                direction = DirectAccel.accel;
                            }


                        }

                        if (gamepadLY != 0)
                        {
                            newMotorPower1 += speedChange(motor1, gamepadLY, direction, factor);
                            newMotorPower2 += speedChange(motor2, gamepadLY, direction, factor);
                            newMotorPower3 += speedChange(motor3, gamepadLY, direction, factor);
                            newMotorPower4 += speedChange(motor4, gamepadLY, direction, factor);
                        }
                    }

                    //Strafe
                    {
                        DirectAccel direction = DirectAccel.decel;
                        double factor = factorSpeedDown;
                        if (gamepadLX > 0) //for positive values

                        {

                            if (gamepadLX - currentSpeedLeft > MIN_Value)
                            {
                                factor = factorSpeedUp;
                                direction = DirectAccel.accel;
                            }


                        }
                        else if (gamepadLX < 0) //for negative values
                        {

                            if (gamepadLX - currentSpeedLeft < - MIN_Value)
                            {
                                factor = factorSpeedUp;
                                direction = DirectAccel.accel;
                            }


                        }
                        if (gamepadLX != 0)
                        {
                            newMotorPower1 += speedChange(motor1, - gamepadLX, direction, factor);
                            newMotorPower2 += speedChange(motor2, - gamepadLX, direction, factor);
                            newMotorPower3 += speedChange(motor3, gamepadLX, direction, factor);
                            newMotorPower4 += speedChange(motor4, gamepadLX, direction, factor);
                        }
                    }

                    //turning
                    {
                        DirectAccel direction = DirectAccel.decel;
                        double factor = factorSpeedDown;
                        if (gamepadRX > 0) //for positive values

                        {

                            if (gamepadRX - currentSpeedLeft > MIN_Value)
                            {
                                factor = factorSpeedUp;
                                direction = DirectAccel.accel;
                            }


                        }
                        else if (gamepadRX < 0) //for negative values
                        {

                            if (gamepadRX - currentSpeedLeft < - MIN_Value)
                            {
                                factor = factorSpeedUp;
                                direction = DirectAccel.accel;
                            }


                        }
                        if (gamepadRX != 0)
                        {
                            newMotorPower1 += speedChange(motor1, gamepadRX, direction, factor);
                            newMotorPower2 += speedChange(motor2, - gamepadRX, direction, factor);
                            newMotorPower3 += speedChange(motor3, gamepadRX, direction, factor);
                            newMotorPower4 += speedChange(motor4, - gamepadRX, direction, factor);
                        }
                    }


                    if (FastMath.abs(newMotorPower1) < MIN_Value)
                    {
                        newMotorPower1 = 0;
                    }
                    if (FastMath.abs(newMotorPower2) < MIN_Value)
                    {
                        newMotorPower2 = 0;
                    }
                    if (FastMath.abs(newMotorPower3) < MIN_Value)
                    {
                        newMotorPower3 = 0;
                    }
                    if (FastMath.abs(newMotorPower4) < MIN_Value)
                    {
                        newMotorPower4 = 0;
                    }


                    newMotorPower1 = Range.clip(newMotorPower1, - 1.0, 1.0);
                    newMotorPower2 = Range.clip(newMotorPower2, - 1.0, 1.0);
                    newMotorPower3 = Range.clip(newMotorPower3, - 1.0, 1.0);
                    newMotorPower4 = Range.clip(newMotorPower4, - 1.0, 1.0);


                    motor1.setPower(newMotorPower1);
                    motor2.setPower(newMotorPower2);
                    motor3.setPower(newMotorPower3);
                    motor4.setPower(newMotorPower4);

                    if (accurateDrive)
                    {
                        factorSpeedUp *= 2;
                        factorSpeedDown *= 2;
                    }

                }

                //Autonomous stuff
                else
                {
                    //fileWriter.write("Auton Started");
                    telemetry.append("firstLoop = " + firstLoop);


                    telemetry.append( (! motor1.isBusy() && ! motor2.isBusy() && ! motor3.isBusy() && ! motor4.isBusy()) + "");


                    telemetry.update();

                    if (! motor1.isBusy() && ! motor2.isBusy() && ! motor3.isBusy() && ! motor4.isBusy())
                    {
                        telemetry.append("In execute loop");
                        telemetry.update();

                        telemetry.append("" + counter);


                        //executeAction(counter);
                        autonomousControl(this.counter);
                        firstLoop = false;
                        this.counter++;

                        distanceFrontLeft = 0;
                        distanceFrontRight = 0;
                        distanceBackLeft = 0;
                        distanceBackRight = 0;
                    }


                    telemetry.print("Action " + (counter - 1) + " complete");
                    //fileWriter.write("Action " + (counter - 1) + " complete");
                }
            }
        }
        catch (Exception e)
        {
            //fileWriter.write("Error occured in DrivingThread:\n" + Arrays.toString(e.getStackTrace()));
            telemetry.print(Arrays.toString(e.getStackTrace()));
        }
    }

    /*private void scanningWithBlueJay()
    {
        skyStones = fieldElementDetector.getSkyStones();

        for (SkyStone detection : skyStones)
        {
            if (detection.y < 111)
            {
                telemetry.append("Skystone Data: X=" + detection.x + ", Y= " + detection.y);

            }

            // TODO: 3/2/2020 Integrate into the detection algo
            if(detection.x < fieldElementDetector.getWidth()/3.0)
            {
                telemetry.speak("Left");
                skystonePosition = Locations.Far;
            }
            else if(detection.x > 2 * fieldElementDetector.getWidth()/3.0)
            {
                telemetry.speak("Right");
                skystonePosition = Locations.Close;

            }
            else
            {
                telemetry.speak("Center");
                skystonePosition = Locations.Center;
            }

        }

        telemetry.update();

        //telemetry.speak(Arrays.toString(skyStones));


        fieldElementDetector.stop();
    }
     */

    private void autonomousControl(int counter)
    {
        switch (counter)
        {
            case 0:
            {
                elevatorThread.elevatorStart();
                telemetry.print("Autonomous started");
                break;
            }

            case 1:
            {
                //scanningWithBlueJay();
                double distance = 0;

                //correctedDrive(distanceUnit.toMm(4), .8);

                switch (skystonePosition)
                {
                    case Close:
                    default:
                    {
                        // TODO: 2/27/2020 5 inches
                        distance = 5;
                        break;
                    }
                    case Center:
                    {
                        distance = -2;
                        break;
                    }
                    case Far:
                    {
                        distance = -9.5;
                        break;
                    }

                }

                lockDrive(distanceUnit.toMm(10),1);
                lockStrafe(distanceUnit.toMm(distance),1);  //correctedStrafe(distanceUnit.toMm(distance),1);
                lockDrive(distanceUnit.toMm(20),1); //correctedDrive(distanceUnit.toMm(20),1);
                elevatorThread.closeClamp();
                try
                {
                    Thread.sleep(750);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
                lockDrive(distanceUnit.toMm(-10),1);   //correctedDrive(distanceUnit.toMm(-10),1);
                elevatorThread.move(elevatorThread.resolveAutonMovement(200, 0));
                try
                {
                    Thread.sleep(750);
                }
                catch (InterruptedException e)
                {
                }

                break;

            }
            case 2:
                {
                    turn(-PI/2.0,.7);   //correctedTurn(-PI/2.0,1,false);
                    lockDrive(distanceUnit.toMm(53),1.0);  //correctedDrive(distanceUnit.toMm(55),1.0);
                    elevatorThread.openClamp();
                    break;
                }

            case 3:
                {
                    lockDrive(distanceUnit.toMm(-57),1.0); //correctedDrive(distanceUnit.toMm(-70),1.0);
                    turn(PI/2.0, 0.7);    //correctedTurn(PI/2.0,1,false);
                    break;
                }
            case 4:
            {
                //scanningWithBlueJay();
                double distance = 0;

                //correctedDrive(distanceUnit.toMm(4), .8);

                switch (skystonePosition)
                {
                    case Close:
                    default:
                    {
                        // TODO: 2/27/2020 5 inches
                        distance = 5;
                        lockStrafe(distanceUnit.toMm(distance),1);    //correctedStrafe(distanceUnit.toMm(distance),1);

                        break;
                    }
                    case Center:
                    {
                        distance = -11.5;
                        lockStrafe(distanceUnit.toMm(distance),1);    //correctedStrafe(distanceUnit.toMm(distance),1);

                        break;
                    }
                    case Far:
                    {
                        turn(PI/6.0,1);
                        break;
                    }


                }

                lockDrive(distanceUnit.toMm(16),1);    //correctedDrive(distanceUnit.toMm(20),1);
                elevatorThread.closeClamp();
                try
                {
                    Thread.sleep(750);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
                lockDrive(distanceUnit.toMm(-10),1);    //correctedDrive(distanceUnit.toMm(-10),1);

                elevatorThread.move(elevatorThread.resolveAutonMovement(200, 0));
                try
                {
                    Thread.sleep(750);
                }
                catch (InterruptedException e)
                {
                }

                break;

            }
            case 5:
            {
                /*elevatorThread.move(elevatorThread.resolveAutonMovement(600, 0));
                try
                {
                    Thread.sleep(750);
                }
                catch (InterruptedException e)
                {
                }
                if (!isRed)
                {
                    correctedStrafe(distanceUnit.toMm(  -12 ),1);
                }
                else
                {
                    correctedStrafe(distanceUnit.toMm(12 ),1);
                }*/
                if(skystonePosition.equals(RobotMovements.Locations.Far))
                {
                    turn(4.0 * -PI/6.0,1);
                }
                else
                    {
                        turn(-PI/2.0,.7); //correctedTurn(-PI/2.0,1,false);
                    }
                lockDrive(distanceUnit.toMm(65),1.0);    //correctedDrive(distanceUnit.toMm(70),1.0);
                elevatorThread.openClamp();

                break;
            }

            case 7:
            {
                if (!foundationMoveRequest)
                {
                    lockDrive(distanceUnit.toMm(-6),1);
                    this.counter = 100000;
                }
                break;
            }
            case 999:
            {
                //foundation movement
                correctedDrive(distanceUnit.toMm(- 6), 0.5);
                elevatorThread.elevatorStart();

                correctedTurn(PI,0.5,false);
                correctedDrive(distanceUnit.toMm(- 12), 0.5);
                grabFoundation();
                try
                {
                    Thread.sleep(225);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
                correctedDrive(distanceUnit.toMm(35), 0.5);
                correctedTurn(- PI / 2, 0.5,false);
                releaseFoundation();
                try
                {
                    Thread.sleep(225);
                }
                catch (InterruptedException ignored)
                {
                }
                correctedStrafe(distanceUnit.toMm(-50), 0.5);
                autonomousControl(9);
                break;

            }
            case 8:
            {
                correctedDrive(-50,1.0);
                correctedTurn(PI,1.0,false);
                break;
            }
            case 9:
            {
                elevatorThread.rulerPark();
                this.counter = 100000;
                break;
            }

            default:
                telemetry.print("Autonomous has ended");
                break;
        }
    }

    /**
     * Execute actions at a given position
     *
     * @param position where the robot is
     */
    private void executeAction(int position) //executes necessary actions at each point throughout the autonomous
    {
        switch (counter)
        {
            case 0:
            {
                telemetry.speak("Autonomous started");
                lockDrive(distanceUnit.toMm(7),1);
            }

            case 1:
            {
                //scanningWithBlueJay();
                double distance = 0;

                //correctedDrive(distanceUnit.toMm(4), .8);
                if (!isRed)
                {
                    switch (skystonePosition)
                    {
                        case Close:
                        {
                            // TODO: 2/27/2020 5 inches
                            distance = 5;
                            break;
                        }
                        case Center:
                        {
                            distance = -8;
                            break;
                        }
                        case Far:
                        {
                            distance = -21;
                            break;
                        }
                    }

                }
                else
                {
                    switch (skystonePosition)
                    {
                        case Close:
                        {
                            break;
                        }
                        case Center:
                        {

                            break;
                        }
                        case Far:
                        {
                            break;
                        }
                    }
                }

                lockStrafe(distanceUnit.toMm(distance),1);
                lockDrive(distanceUnit.toMm(20),1);
                doStop();
                break;

            }
            case 5:
            {

            }
            case 6:
            {

            }
            case 7:
            {

            }
            case 8:
            {
                while(distanceTo()>50)
                {
                    motor1.setPower(0.5);
                    motor2.setPower(0.5);
                    motor3.setPower(0.5);
                    motor4.setPower(0.5);
                }

                if(distanceTo()<50)
                {
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                    motor4.setPower(0);
                }

                grabFoundation();
            }
            default:
                telemetry.print("Autonomous has ended");
                break;
        }
    }

    /*
     * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
     */
    @Deprecated
    private void initVuforia()
    {


        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        robot.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //fileWriter.write("Vuforia Started");

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Creates Tfod for object detection
     */
    @Deprecated
    private void initTfod()
    {
        //creates a tfod object which is using the tfodMonitor
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //creates a new parameters object
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        //Sets the minimum confidence for the program to read a block to the values stored in the Team6438HardwareMap
        tfodParameters.minimumConfidence = Team6438ChassisHardwareMapCurrent.confidence;

        //sets the tfod object equal to a new tfod object with parameters
        robot.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, robot.vuforia);

        //Loads this years assets
        robot.tfod.loadModelFromAsset(robot.TFOD_MODEL_ASSET, robot.LABEL_STONE, robot.LABEL_SKYSTONE);

        //fileWriter.write("TFOD Initialized");

    }

    /**
     * Scans the skystone
     *
     * @return position of Skystone
     */
    @Deprecated
    private int newScan()
    {
        if (robot.tfod != null)
        {
            boolean stoneFound = false;

            List<Recognition> recognitions = robot.tfod.getUpdatedRecognitions();
            while (recognitions != null)//&& runtime.milliseconds() < 1500)
            {
                recognitions = robot.tfod.getUpdatedRecognitions();

                for (Recognition recognition : recognitions)
                {
                    if (recognition.getLabel().equals(robot.LABEL_SKYSTONE))
                    {
                        return 0;   //Skystone is the first(closest) stone
                    }
                    else
                    {
                        List<Recognition> recognitions2 = robot.tfod.getUpdatedRecognitions();
                        while (recognitions2 != null)
                        {
                            for (Recognition recognition2 : recognitions2)
                            {
                                if (recognition2.getLabel().equals(robot.LABEL_SKYSTONE))
                                {
                                    return 1; //Skystone is the center stone
                                }
                                else
                                {
                                    return 2; //Skystone is the last(farthest) stone
                                }
                            }
                        }
                    }
                }
            }
        }
        return 1;
    }

    @Deprecated
    private int scanSkystone()
    {
        int position = - 1; //Returns -1 if the block isn't detected
        boolean rightFound = false;
        boolean centerFound = false;

        //Call the init TFod method to get block detection ready

        /*if (ClassFactory.getInstance().canCreateTFObjectDetector())
        {
            initTfod();
        }

         */


        //Activate Tensor Flow Object Detection.
        if (robot.tfod != null)
        {


            boolean stoneFound = false;


            while (true)//&& runtime.milliseconds() < 1500)
            {
                //telemetry.print("no detection");
                // getUpdatedRecognitions() will return null if no new information is available since the last time that call was made.
                List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null)
                {

                    for (Recognition recognition : updatedRecognitions)
                    {
                        if (! isRed)
                        {
                            telemetry.print("" + recognition.getLeft());
                            if (recognition.getLabel().equals(robot.LABEL_SKYSTONE))
                            {
                                if (recognition.getLeft() > 100 && recognition.getLeft() < 250)    //Skystone in the center
                                {
                                    telemetry.append("right");
                                    telemetry.append("Left Bounds = " + recognition.getLeft());
                                    position = 1;
                                    if (robot.tfod != null)
                                    {

                                        robot.tfod.shutdown();
                                        //vuforiaCamera1.killVuforia();

                                    }
                                    //fileWriter.write(("Block Left Bounds = " + recognition.getLeft()));
                                    return position;
                                }
                                else if (recognition.getLeft() <= 100)  //Skystone on the right
                                {
                                    telemetry.append("center");
                                    telemetry.append("Left Bounds = " + recognition.getLeft());
                                    position = 0;

                                    if (robot.tfod != null)
                                    {

                                        robot.tfod.shutdown();
                                        //vuforiaCamera1.killVuforia();

                                    }
                                    //fileWriter.write("Block Left Bounds = " + recognition.getLeft());
                                    return position;
                                }
                            }
                            else
                            {
                                if (recognition.getLeft() > 100 && recognition.getLeft() < 200 && ! rightFound)    //Skystone in the center
                                {
                                    rightFound = true;
                                }

                                if (recognition.getLeft() <= 100 && ! centerFound)    //Skystone in the center
                                {
                                    centerFound = true;
                                }

                                if (rightFound && centerFound)
                                {
                                    robot.tfod.shutdown();
                                    //fileWriter.write("Odd-Man successful");
                                    return 2;
                                }
                            }
                        }
                        else
                        {
                            if (recognition.getLabel().equals(robot.LABEL_SKYSTONE))
                            {
                                telemetry.print("" + recognition.getLeft());

                                if (recognition.getLeft() > 300 && recognition.getLeft() < 600)    //Skystone in the center
                                {
                                    telemetry.append("left");
                                    telemetry.append("Left Bounds = " + recognition.getLeft());
                                    position = 0;
                                    if (robot.tfod != null)
                                    {
                                        robot.tfod.shutdown();
                                        //vuforiaCamera1.killVuforia();
                                    }
                                    return position;
                                }
                                else if (recognition.getLeft() <= 300)  //Skystone on the right
                                {
                                    telemetry.append("center");
                                    telemetry.append("Left Bounds = " + recognition.getLeft());
                                    position = 1;

                                    if (robot.tfod != null)
                                    {
                                        robot.tfod.shutdown();
                                        //vuforiaCamera1.killVuforia();
                                    }
                                    return position;
                                }
                            }
                            else
                            {

                                if (updatedRecognitions.size() >= 2)
                                {
                                    if (recognition.getLeft() > 300 && recognition.getLeft() < 600)    //Skystone in the center
                                    {
                                        rightFound = true;
                                    }

                                    if (recognition.getLeft() <= 300)    //Skystone in the center
                                    {
                                        centerFound = true;
                                    }

                                    if (rightFound && centerFound)
                                    {
                                        robot.tfod.shutdown();
                                        return 2;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        return position;
    }

    /*private void findSkystone(OpenCvCamera webcam)
    {
        OurSkystoneDetector skyStoneDetector;
        Locations position = null;
        skyStoneDetector = new OurSkystoneDetector();
        webcam.setPipeline(skyStoneDetector);

        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
        while (position == null)
        {
            if (isRed)
            {
                if (skyStoneDetector.getScreenPosition().x < 150)
                {
                    position = Locations.Close;
                }
                else if (skyStoneDetector.getScreenPosition().x > 150 && skyStoneDetector.getScreenPosition().x < 200)
                {
                    position = Locations.Center;
                }
                else if (skyStoneDetector.getScreenPosition().x > 200 && skyStoneDetector.getScreenPosition().x < 250)
                {
                    position = Locations.Far;
                }
            }
            else
            {
                if (skyStoneDetector.getScreenPosition().x < 50)
                {
                    position = Locations.Far;
                }
                else if (skyStoneDetector.getScreenPosition().x > 55 && skyStoneDetector.getScreenPosition().x < 83)
                {
                    position = Locations.Center;
                }
                else if (skyStoneDetector.getScreenPosition().x > 83 && skyStoneDetector.getScreenPosition().x < 120)
                {
                    position = Locations.Close;
                }
            }
        }

        telemetry.speak(position.toString());

        skystonePosition = position;
    }

    public enum Locations {Close, Center, Far}
    */


    @Deprecated
    private void scanningMonitor()
    {
        activateTfod();
        robot.tfod.getUpdatedRecognitions();
        for (Recognition recog : robot.tfod.getRecognitions())
        {
            if (recog.getLabel().equals(robot.LABEL_SKYSTONE))
            {
                objectDetected = true;
            }
        }
        robot.tfod.deactivate();
    }

    @Deprecated
    private void activateTfod()
    {
        if (robot.tfod != null)
        {
            robot.tfod.activate();
        }
    }

    private boolean scanningStrafe()
    {

        if (objectDetected)
        {
            //lockStrafe(radiusMM, 1);
            correctedStrafe(radiusMM, 1);
            scanDistance += radiusMM;
            return true;
        }
        else
        {
            //lockStrafe(radiusMM*2, 1);
            correctedStrafe(radiusMM * 2, 1);
            scanDistance += radiusMM * 2;
            return false;

        }

    }

    /**
     * If the motor is not moving set it a position
     *
     * @param motor    motor to assign position
     * @param power
     * @param distance distance to traverse
     */
    private void assignPosition(DcMotor motor, double power, double distance)
    {

        double target = distance;
        motor.setTargetPosition((int) target);
    }

    void grabFoundation()
    {
        robot.foundationL.setPosition(1);
        robot.foundationR.setPosition(0);
    }

    void releaseFoundation()
    {
        robot.foundationL.setPosition(.5);
        robot.foundationR.setPosition(.5);
    }

    void midFoundation()
    {
        robot.foundationL.setPosition(.9);
        robot.foundationR.setPosition(.1);
    }

    /**
     * @param angle
     * @param power
     * @param includePhoneGyro
     */
    private void correctedTurn(double angle, double power, boolean includePhoneGyro)
    {
        double error;
        if (angle > 0)
        {
            oldHeadingIMU = getHeadingIMU();


            turn(angle, power);

            newHeadingIMU = getHeadingIMU();
            double error1;

            error = (angle - checkAngles());

            telemetry.append("old " + oldHeadingIMU);
            telemetry.append("new " + newHeadingIMU);
            telemetry.append("Turn " + angle);
            telemetry.append("Delta " + checkAngles());
            telemetry.append("Error IMU gyro is: " + error);
            telemetry.append("Err% is: " + (error / angle * 100));
            telemetry.update();
        }
        else
        {
            oldHeadingIMU = - getHeadingIMU();


            turn(angle, power);

            newHeadingIMU = - getHeadingIMU();
            double error1;

            error = (- angle - checkAngles());

            telemetry.append("old " + oldHeadingIMU);
            telemetry.append("new " + newHeadingIMU);
            telemetry.append("Turn " + angle);
            telemetry.append("Delta " + checkAngles());
            telemetry.append("Error IMU gyro is: " + error);
            telemetry.append("Err% is: " + (error / angle * 100));
            telemetry.update();

        }

        if (abs(error) > gain)
        {

            correctedTurn(error, power, includePhoneGyro);
        }

    }

    /**
     * @param power
     */
    private void allDrive(double power)
    {
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
        motor4.setPower(power);

    }

    /**
     * @param angle
     * @param power
     */
    private void turn(final double angle, final double power)
    {
        // telemetry.print("Turning");

        float distance = (float) (angle * Team6438ChassisHardwareMapCurrent.radiusMM);


        assignPosition(motor1, motorMovementEncoder(- distance, motor1));
        assignPosition(motor2, motorMovementEncoder(distance, motor2));
        assignPosition(motor3, motorMovementEncoder(- distance, motor3));
        assignPosition(motor4, motorMovementEncoder(distance, motor4));

        allDrive(power);
        while (abs(motor1.getCurrentPosition() - motor1.getTargetPosition()) > 5)
        {
           /* telemetry.append("Turning  " + angle);
            telemetry.append("FL Target: " + motor1.getTargetPosition());
            telemetry.append("FR Target: " + motor2.getTargetPosition());
            telemetry.append("BL Target: " + motor3.getTargetPosition());
            telemetry.append("BR Target: " + motor4.getTargetPosition());
            telemetry.append("FL Current: " + motor1.getCurrentPosition());
            telemetry.append("FR Current: " + motor2.getCurrentPosition());
            telemetry.append("BL Current: " + motor3.getCurrentPosition());
            telemetry.append("BR Current: " + motor4.getCurrentPosition());
            telemetry.update();*/

        }

        try {
            Thread.sleep(75);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

    /**
     * @param motor
     * @param distance
     */
    private void assignPosition(DcMotor motor, double distance)
    {

        motor.setTargetPosition((int) distance);

        /*telemetry.append("Cleared motor to move: " + motor.getDeviceName());
        telemetry.append(motor.getTargetPosition() + "");
        telemetry.update();*/

    }

    /**
     * @param position
     * @param motor
     *
     * @return
     */
    private int motorMovementEncoder(double position, @NotNull DcMotor motor)
    {
        int currentPosition = motor.getCurrentPosition();

        currentPosition += (int) round(position * Team6438ChassisHardwareMapCurrent.hexCPMM);

        /* telemetry.append("" + motor + " Target : " + currentPosition);*/

        return currentPosition;

    }

    /**
     * Locks the Driving as the robot moves towards this position
     *
     * @param distance distance to travel
     * @param power
     */
    private void lockDrive(double distance, final double power)
    {
        assignPosition(motor1, motorMovementEncoder(round(distance), motor1));
        assignPosition(motor2, motorMovementEncoder(round(distance), motor2));
        assignPosition(motor3, motorMovementEncoder(round(distance), motor3));
        assignPosition(motor4, motorMovementEncoder(round(distance), motor4));

        allDrive(power);
        while (motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && motor4.isBusy())
        {
            System.out.println("here");
        }

        try {
            Thread.sleep(75);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

    /**
     * @param distance
     * @param power
     */
    private void lockStrafe(double distance, final double power)
    {
        if (! isRed)
        {
            distance *= - 1;
        }

        assignPosition(motor1, motorMovementEncoder(round(- distance), motor1));
        assignPosition(motor2, motorMovementEncoder(round(- distance), motor2));
        assignPosition(motor3, motorMovementEncoder(round(distance), motor3));
        assignPosition(motor4, motorMovementEncoder(round(distance), motor4));

        allDrive(power);
        while (motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && motor4.isBusy())
        {
            System.out.println("here");
        }

        try {
            Thread.sleep(75);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * @param driveDistance
     * @param oldHeadingIMU
     *
     * @return
     */
    private drivingThread.MovementVector calcDeviation(double driveDistance, double oldHeadingIMU)
    {
        double deviationDistance;
        double deviationAngle;

        newHeadingIMU = getHeadingIMU();

        deviationAngle = AngleUnit.normalizeRadians(newHeadingIMU - oldHeadingIMU);

        deviationDistance = 2 * driveDistance * sin(deviationAngle / 2.0);

        return new drivingThread.MovementVector(deviationDistance, - deviationAngle);
    }

    /**
     * @param movementVector
     * @param power
     * @param driveType
     */
    private void deviationCorrection(final drivingThread.MovementVector movementVector, final double power, drivingThread.DriveType driveType)
    {
        if (driveType == drivingThread.DriveType.strafe)
        {
            correctedStrafe(movementVector.getDistance(), power);
        }
        else if (driveType == drivingThread.DriveType.drive)
        {
            correctedDrive(movementVector.getDistance(), power);
        }
        else
        {
            throw new IllegalArgumentException("Illegal movement type " + driveType);
        }

        correctedTurn(movementVector.getAngle(), power, false);
    }

    /**
     * @param driveDistance
     * @param oldHeadingIMU
     * @param power
     * @param driveType
     */
    private void rectifyDeviations(double driveDistance, double oldHeadingIMU, double power, drivingThread.DriveType driveType)
    {
        drivingThread.MovementVector movementVector = calcDeviation(driveDistance, oldHeadingIMU);
        if (FastMath.abs(movementVector.getDistance()) <= 1 && FastMath.abs(movementVector.getAngle()) <= gain)
        {
            return;
        }
        deviationCorrection(movementVector, power, driveType);
    }

    /**
     * @param distance
     * @param power
     */
    private void correctedStrafe(final double distance, final double power)
    {
        double power1 = Range.scale(power, 0.0, 1.0, 0.0, .80);
        oldHeadingIMU = getHeadingIMU();

        lockStrafe(distance, power1);

        rectifyDeviations(distance, oldHeadingIMU, power1, drivingThread.DriveType.drive);

    }

    private double distanceTo()
    {
        return frontSensor.getDistance(DistanceUnit.MM);
    }

    /**
     * @param distance
     * @param power
     */
    private void correctedDrive(final double distance, final double power)
    {
        oldHeadingIMU = getHeadingIMU();

        lockDrive(distance, power);

        rectifyDeviations(distance, oldHeadingIMU, power, drivingThread.DriveType.strafe);
    }

    /**
     * @return
     */
    private double getHeadingIMU()
    {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    private double checkAngles()
    {
        double deltaAngle = newHeadingIMU - oldHeadingIMU;


        return deltaAngle;
    }



    private enum DriveType {
        strafe, drive
    }

    public enum ScanPosition {left, center, right}

    /**
     * enum with which direction the acceleration is in relative to the current power
     */
    private enum DirectAccel {accel, decel}

    private class MovementVector {
        private final double distance;
        private final double angle;

        MovementVector(double distance, double angle)
        {

            this.distance = distance;
            this.angle = angle;
        }

        public double getDistance()
        {
            return distance;
        }

        public double getAngle()
        {
            return angle;
        }
    }
}



