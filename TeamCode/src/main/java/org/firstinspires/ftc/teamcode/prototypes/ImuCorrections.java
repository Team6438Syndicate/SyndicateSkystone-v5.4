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

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Team6438ChassisHardwareMapCurrent;
import org.firstinspires.ftc.teamcode.odometry.Telemetry;
import org.jetbrains.annotations.NotNull;

import static org.apache.commons.math3.util.FastMath.PI;
import static org.apache.commons.math3.util.FastMath.abs;
import static org.apache.commons.math3.util.FastMath.round;
import static org.apache.commons.math3.util.FastMath.sin;


@SuppressWarnings("JavaDoc")
public class ImuCorrections implements Runnable {
    private final float[] mAccelerometerReading = new float[3];
    private final float[] mMagnetometerReading = new float[3];
    private final float[] mRotationMatrix = new float[9];
    private final float[] mOrientationAngles = new float[3];
    private double oldHeadingGyro = 0;
    private double newHeadingGyro = 0;


    private Team6438ChassisHardwareMapCurrent robot = null;
    private Telemetry telemetry;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private float[] angleValue;
    private boolean turnDev;
    private boolean doStop = false;
    private float timestamp;
    private boolean isRed;

    ImuCorrections(Team6438ChassisHardwareMapCurrent team6438ChassisHardwareMapCurrent, Telemetry telemetry, DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4, double gain, final boolean isRed)
    {
        this.isRed = isRed;
        turnDev = true;
        this.robot = team6438ChassisHardwareMapCurrent;

        this.telemetry = telemetry;
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.motor3 = motor3;
        this.motor4 = motor4;
        this.gain = gain;

        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 10);


    }

    ImuCorrections(Context context, Telemetry telemetry)
    {
        this.telemetry = telemetry;
        turnDev = false;
        //this.robot = team6438ChassisHardwareMapCurrent;

        //this.telemetry = telemetry;

        //robot.imu.startAccelerationIntegration(new Position(),new Velocity(),10);


        SensorManager sensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        /*Sensor sensor = sensorManager.getDefaultSensor(Sensor.TYPE_GEOMAGNETIC_ROTATION_VECTOR);
        sensorManager.registerListener(new SensorEventListener() {
            @Override
            public void onSensorChanged(final SensorEvent event)
            {

            }

            @Override
            public void onAccuracyChanged(final Sensor sensor, final int accuracy)
            {

            }
        }, sensor, SensorManager.SENSOR_DELAY_FASTEST);*/
        SensorEventListener sensorEventListener = new SensorEventListener() {
            @Override
            public void onSensorChanged(final SensorEvent event)
            {
                if (event.sensor.getType() == (Sensor.TYPE_ACCELEROMETER))
                {
                    System.arraycopy(event.values, 0, mAccelerometerReading,
                            0, mAccelerometerReading.length);
                }
                else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
                {
                    System.arraycopy(event.values, 0, mMagnetometerReading,
                            0, mMagnetometerReading.length);
                }
            }

            @Override
            public void onAccuracyChanged(final Sensor sensor, final int accuracy)
            {

            }
        };
        Sensor accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        if (accelerometer != null)
        {
            sensorManager.registerListener(sensorEventListener, accelerometer,
                    SensorManager.SENSOR_DELAY_GAME, SensorManager.SENSOR_DELAY_GAME);
        }
        Sensor magneticField = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        if (magneticField != null)
        {
            sensorManager.registerListener(sensorEventListener, magneticField,
                    SensorManager.SENSOR_DELAY_GAME, SensorManager.SENSOR_DELAY_GAME);
        }


    }

    synchronized void doStop()
    {
        this.doStop = true;

    }

    private synchronized boolean keepRunning()
    {
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
        DistanceUnit distanceUnit =  DistanceUnit.INCH;

            if (turnDev)
            {
               /* telemetry.append(robot.imu.getPosition().toString());
                telemetry.update();*/
                correctedTurn(PI / 2, 1.0, false);
                try
                {
                    Thread.sleep(2000);
                }
                catch (InterruptedException ignored)
                {

                }
                telemetry.append(robot.imu.getPosition().toString());
                telemetry.update();
               correctedStrafe(distanceUnit.toMm(20),1);
                try
                {
                    Thread.sleep(2000);
                }
                catch (InterruptedException ignored)
                {

                }
                telemetry.append(robot.imu.getPosition().toString());
                telemetry.update();
                correctedDrive(distanceUnit.toMm(-15),1);
                try
                {
                    Thread.sleep(2000);
                }
                catch (InterruptedException ignored)
                {

                }
                telemetry.append(robot.imu.getPosition().toString());
                telemetry.update();
                correctedDrive(distanceUnit.toMm(15),1);
                try
                {
                    Thread.sleep(2000);
                }
                catch (InterruptedException ignored)
                {

                }
                telemetry.append(robot.imu.getPosition().toString());
                telemetry.update();
                correctedStrafe(distanceUnit.toMm(-20),1);
                try
                {
                    Thread.sleep(2000);
                }
                catch (InterruptedException ignored)
                {

                }
                telemetry.append(robot.imu.getPosition().toString());
                telemetry.update();
                correctedTurn(-PI / 2.0, 1.0, false);
                try
                {
                    Thread.sleep(2000);
                }
                catch (InterruptedException ignored)
                {

                }
                telemetry.append("Done");
                telemetry.append(robot.imu.getPosition().toString());
                telemetry.update();


        }
    }

    /**
     *
     */

    private double oldHeadingIMU;

    private double newHeadingIMU;
    private double gain;
    /**
     *
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
            oldHeadingIMU = -getHeadingIMU();


            turn(angle, power);

            newHeadingIMU = -getHeadingIMU();
            double error1;

            error = (-angle - checkAngles());

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
     *
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
     *
     * @param angle
     * @param power
     */
    private void turn(final double angle, final double power)
    {
       // telemetry.print("Turning");

        float distance = (float) (angle * Team6438ChassisHardwareMapCurrent.radiusMM);


        assignPosition(motor1, motorMovementEncoder(-distance, motor1));
        assignPosition(motor2, motorMovementEncoder(distance, motor2));
        assignPosition(motor3, motorMovementEncoder(-distance, motor3));
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


    }

    /**
     *
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
     *
     * @param position
     * @param motor
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

    }

    /**
     *
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


    }

    /**
     *
     * @param driveDistance
     * @param oldHeadingIMU
     * @return
     */
    private MovementVector calcDeviation(double driveDistance, double oldHeadingIMU)
    {
        double deviationDistance;
        double deviationAngle;

        newHeadingIMU = getHeadingIMU();

        deviationAngle = AngleUnit.normalizeRadians(newHeadingIMU - oldHeadingIMU);

        deviationDistance = 2 * driveDistance * sin(deviationAngle / 2.0);

        return new MovementVector(deviationDistance, -deviationAngle);
    }

    /**
     *
     * @param movementVector
     * @param power
     * @param driveType
     */
    private void deviationCorrection(final MovementVector movementVector, final double power, DriveType driveType)
    {
        if (driveType == DriveType.strafe)
        {
            correctedStrafe(movementVector.getDistance(), power);
        }
        else if(driveType == DriveType.drive)
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
     *
     */
    private enum DriveType
    {
        strafe,drive
    }

    /**
     *
     * @param driveDistance
     * @param oldHeadingIMU
     * @param power
     * @param driveType
     */
    private void rectifyDeviations(double driveDistance, double oldHeadingIMU, double power, DriveType driveType)
    {
        MovementVector movementVector = calcDeviation(driveDistance, oldHeadingIMU);
        if(FastMath.abs(movementVector.getDistance()) <= 1 && FastMath.abs(movementVector.getAngle()) <= gain)
        {
            return;
        }
        deviationCorrection(movementVector,power, driveType);
    }

    /**
     *  @param distance
     * @param power
     */
    private void correctedStrafe(final double distance, final double power)
    {
        double power1 = Range.scale(power,0.0,1.0,0.0,.80);
        oldHeadingIMU = getHeadingIMU();

        lockStrafe(distance,power);

        rectifyDeviations(distance,oldHeadingIMU,power,DriveType.drive);

    }

    /**
     *  @param distance
     * @param power
     */
    private void correctedDrive(final double distance, final double power)
    {
        oldHeadingIMU = getHeadingIMU();

        lockDrive(distance,power);

        rectifyDeviations(distance,oldHeadingIMU,power,DriveType.strafe);
    }

    /**
     *
     * @return
     */
    private double getHeadingIMU()
    {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    private double checkAngles()
    {
        double deltaAngle = newHeadingIMU - oldHeadingIMU;

        /*if (deltaAngle < -PI/2.0)
            deltaAngle += 2.0*PI;
        else if (deltaAngle > PI/2.0)
            deltaAngle -= 2.0*PI;*/
        return deltaAngle;
    }

    /**
     *
     */
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
