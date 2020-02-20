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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;
import java.util.Objects;

import static org.apache.commons.math3.util.FastMath.acos;

public class ArmControlThread implements Runnable {
    private DcMotor motor1;
    private Servo elbowLS;
    private Servo elbowRs;
    private Servo hand;
    private Servo wristS;
    private DistanceSensor distanceSensor;
    private int mills;
    private Gamepad gamepad;
    private boolean userControllable;
    private boolean doStop = false;

    private boolean grabbing = false;
    private boolean stacking = false;

    //private float HEIGHT_OFFSET = -60.2f;
    private double towerHeight = 50.4 + 0;


    private HashMap<LocationType,Double> storage = new HashMap<>();
    private int rangeAccuracy;



    public ArmControlThread(@NotNull DcMotor motor1, Servo elbowLS, Servo elbowRs, Servo hand, Servo wristS, DistanceSensor distanceSensor, int mills, Gamepad gamepad) {

        this.motor1 = motor1;
        this.elbowLS = elbowLS;
        this.elbowRs = elbowRs;
        this.hand = hand;
        this.wristS = wristS;
        this.distanceSensor = distanceSensor;
        this.mills = mills;
        this.userControllable = true;
        this.gamepad = gamepad;
        this.rangeAccuracy = rangeAccuracy;

        this.userControllable = true;
    }

    // TODO: 11/12/2019 implement this constructor for autonomous movement but current is MIA
    ArmControlThread(@NotNull DcMotor motor1,Servo elbowLS, Servo elbowRs, Servo hand, Servo wristS, DistanceSensor distanceSensor, int mills, int rangeAccuracy) {
        this.motor1 = motor1;
        this.elbowLS = elbowLS;
        this.elbowRs = elbowRs;
        this.hand = hand;
        this.wristS = wristS;
        this.distanceSensor = distanceSensor;
        this.mills = mills;
        this.userControllable = false;
        this.rangeAccuracy = rangeAccuracy;
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

        elbowRs.setDirection(Servo.Direction.FORWARD);
        elbowLS.setDirection(Servo.Direction.REVERSE);
        hand.scaleRange(.25,.9);
        wristS.scaleRange(0,1.0);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hand.setPosition(0);

        while (keepRunning())
        {

            if (!motor1.isBusy())

            {
                motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //this is the distance to the tower in mm
                //How far away the shoulder axle is from the distance sensor in mm
                float DISTANCE_OFFSET = 45.7f;
                double towerDistance = distanceSensor.getDistance(DistanceUnit.MM) + DISTANCE_OFFSET;

                if (userControllable)

                {
                    if (gamepad.a)
                    {
                        armMovement((float) towerDistance, (float) towerHeight, .2);

                        towerHeight += 110f;

                        stacking = true;
                    }
                    else if (gamepad.x)
                    {
                        setShoulder(0, 0.3);
                        elbowRs.setPosition(0);
                        elbowLS.setPosition(0);
                        wristS.setPosition(0);
                        hand.setPosition(0.12);
                    }
                    else if (gamepad.y && !grabbing && !stacking)
                    {
                        hand.setPosition(0.7);

                        if (motor1.getCurrentPosition() < 700)
                        {
                            elbowRs.setPosition(0.35);
                            elbowLS.setPosition(0.35);
                        }

                        setShoulder(800, .2);

                        wristS.setPosition(0.1155);

                        elbowRs.setPosition(0.1);
                        elbowLS.setPosition(0.1);

                        hand.setPosition(0);

                        grabbing = true;
                    }
                    else if (!grabbing && !stacking)
                    {
                        wristS.setPosition(gamepad.left_stick_y);

                        elbowRs.setPosition(gamepad.right_stick_y);
                        elbowLS.setPosition(gamepad.right_stick_y);
                    }

                    if (gamepad.b)
                    {
                        hand.setPosition(0.7);
                        grabbing = false;
                        stacking = false;
                    }
                    else if (hand.getPosition() != 0)
                    {
                        hand.setPosition(0);
                    }

                    if (gamepad.dpad_up)
                    {
                        rotateShoulder(50, 0.3);
                    }
                    else if (gamepad.dpad_down)
                    {
                        rotateShoulder(-50, 0.3);
                    }
                }

            }
            try {

                Thread.sleep(mills);
            } catch (InterruptedException ignored) {

            }
        }
    }



    /**
     * Move the 3 axis arm into place
     *
     * @param targetX  X distance from location
     * @param targetY  Y distance from location
     * @param power    power level
     */

    @SuppressWarnings("ConstantConditions")
    public void armMovement(float targetX, float targetY, double power)
    {
        calcArmAngles(targetX,targetY);

        double targetElbow = storage.get(LocationType.elbowServoPositionType);
        double targetWrist = storage.get(LocationType.wristServoPositionType);

        int targetShoulder = storage.get(LocationType.shoulderMotorTickType).intValue();


        elbowLS.setPosition(targetElbow);
        elbowRs.setPosition(targetElbow);

        wristS.setPosition(targetWrist);

        motor1.setTargetPosition(1288-targetShoulder);

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(power);

        while( FastMath.abs(motor1.getCurrentPosition() - motor1.getTargetPosition()) > 50)
        {
            motor1.setPower(power);
        }

        motor1.setPower(0);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }




    /**
     * Does the Trigonometry for the arm movement method
     *
     * @param tX target x value; horizontal distance away from the tower
     * @param tY target y value; vertical distance away from tower
     *
     *
     */
    private void calcArmAngles(float tX, float tY) {
        storage.clear();
        float FL = Team6438ChassisHardwareMapCurrent.FOREARM_LENGTH;
        float SL = Team6438ChassisHardwareMapCurrent.SHOULDER_LENGTH;

        float tD = (float)FastMath.hypot(tX, tY);

        if (tD < FL+SL)
        {
            double elbowAngle = lawOfCosines(tD, SL, FL);

            double armInterior = FastMath.asin(SL * FastMath.sin(elbowAngle) / tD);

            double shoulderPiece = FastMath.abs(FastMath.atan(tY / tX));

            double PI = FastMath.PI;

            double wristAngle = PI/2 - armInterior + shoulderPiece;

            double shoulderAngle = shoulderPiece + PI - elbowAngle - armInterior;

            //wristAngle -= PI/2.0;     //Makes the wrist change from parallel to the floor to perpendicular

            elbowAngle = Range.scale(elbowAngle,0, PI,0.0,1.0);

            storage.put(LocationType.elbowServoPositionType, elbowAngle);

            wristAngle = Range.scale(wristAngle,0, PI * 3.0/2.0,0.0,1.0);


            storage.put(LocationType.wristServoPositionType,  wristAngle);

            //The encoder tick rate for the shoulder
            float TICKS_PER_DEGREE = 1425.2f / 180f;
            storage.put(LocationType.shoulderMotorTickType, degreesToTicks(FastMath.toDegrees(shoulderAngle), TICKS_PER_DEGREE));
        }
    }

     private double lawOfCosines(double hyp, double a, double b) {
        double angle = acos((hyp*hyp - a*a - b*b) / (- 2.0f * a * b));
        return  angle;
    }

    private int setAngle(@NotNull DcMotor motor)
    {
        int angle = Objects.requireNonNull(storage.get(LocationType.shoulderMotorTickType)).intValue();

        angle = motor.getCurrentPosition() - angle;

        if(angle < 0)
        {
            angle = motor.getCurrentPosition() + angle;
        }
        return angle;
    }

    private double setAngle(@NotNull Servo servo, LocationType locationType)
    {
        double angle = storage.get(locationType);

        angle = servo.getPosition() - angle;

        if(angle < 0)
        {
            angle = servo.getPosition() + angle;
        }

        angle = Range.clip(angle,0.0,1.0);
        return angle;
    }

    public void setShoulder(int position, double power)
    {
        motor1.setTargetPosition(position);

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(power);

        while( FastMath.abs(motor1.getCurrentPosition() - motor1.getTargetPosition()) > 20)
        {
            motor1.setPower(power);
        }

        motor1.setPower(0);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rotateShoulder(int distance, double power)
    {
        motor1.setTargetPosition(motor1.getCurrentPosition() + distance);

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(power);

        while( FastMath.abs(motor1.getCurrentPosition() - motor1.getTargetPosition()) > 20)
        {
            motor1.setPower(power);
        }

        motor1.setPower(0);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetShoulderEncoder()
    {
        while (motor1.isBusy())
        {

        }
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setElbow(double position)
    {
        elbowLS.setPosition(elbowLS.getPosition() + position);
        elbowRs.setPosition(elbowRs.getPosition() + position);
    }

    public void setWrist(double position)
    {
        wristS.setPosition(wristS.getPosition() + position);
    }

    public void toggleGrab()
    {

    }


    /**
     * Convert degrees to ticks for the encoders
     *
     * @param degrees        angle measurement
     * @param conversionRate constant for the conversion depends on the motor and it's gearing
     *
     * @return the tick number
     */
    @Contract(pure = true)
    private double degreesToTicks(double degrees, float conversionRate)
    {
        return FastMath.round (degrees * conversionRate);
    }

    private enum LocationType
    {
        shoulderMotorTickType, elbowServoPositionType, wristServoPositionType;
    }

}
