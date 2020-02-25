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

/**
 * Name: Team6438HardwareMap
 * Purpose: This class contains maps for all hardware on the robot
 * To reference it you need to create an instance of the Team6438HardwareMap class
 * Author: Matthew Batkiewicz, Matthew Kaboolian, Bradley Abelman, David Stekol
 * Contributors:
 * Creation: 8/29/19
 * Last Edit: 8/29/19
 * Additional Notes:
 * ^^Spreadsheet to check our hardware
 * Phones set up on Sep 23rd
 **/
package org.firstinspires.ftc.teamcode.markmattcode;

//Imports

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class SimplifiedHardwareMap
{

    //Vuforia Variables

    static final String VUFORIA_KEY = "ATEEWHn/////AAABmXzvuqxXZkYkr3AeTQT4Qg0P3tudpoBP/Rp2Xyw3zNlZYk+ZI5Jp/yo8TDf62o+UjdBvoe0LP5nNDqFESCtSImOG2WRuMkoESAyhSVzMU0hY53dWb4l0s7mCe+xqqT8i0r9pPdav7N7RiGHG7WYoIBXrQeyz+NEq8TLYTTCXmZMFgPeEU30Nb+t4JikoNMr0X0Ej6y1vG+7EX3O9KI8RXoPYbBmPzvX5uVvWBNg2J0g0SBiZUXa8pQOCxi0QyHyNUiwvV5WKnM2jncg+eI7im5s+k4yn6Xjaeecg6q9IT45YNvbhV4PM/LbwGQTKBf0AOCM/qL7tz7evypWw5uK15BayqAitBLy7Sr0SvIjYMjPg";



    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public final int     CPI        =         (int) ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                            (WHEEL_DIAMETER_INCHES * 3.1415));
    private final String vuforia = "ATEEWHn/////AAABmXzvuqxXZkYkr3AeTQT4Qg0P3tudpoBP/Rp2Xyw3zNlZYk+ZI5Jp/yo8TDf62o+UjdBvoe0LP5nNDqFESCtSImOG2WRuMkoESAyhSVzMU0hY53dWb4l0s7mCe+xqqT8i0r9pPdav7N7RiGHG7WYoIBXrQeyz+NEq8TLYTTCXmZMFgPeEU30Nb+t4JikoNMr0X0Ej6y1vG+7EX3O9KI8RXoPYbBmPzvX5uVvWBNg2J0g0SBiZUXa8pQOCxi0QyHyNUiwvV5WKnM2jncg+eI7im5s+k4yn6Xjaeecg6q9IT45YNvbhV4PM/LbwGQTKBf0AOCM/qL7tz7evypWw5uK15BayqAitBLy7Sr0SvIjYMjPg";


    //Motor  Declaration
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;
    public DcMotor liftMotor = null;
    public DcMotor tensionMotor = null;

    //Servo Declaration
    public  Servo clampL = null;
    public  Servo clampR = null;
    public  Servo foundationL = null;
    public  Servo foundationR = null;

    //Webcam mapping
    public WebcamName camera = null;

    //Sensor Mapping
    public DistanceSensor sensorFront;
    public BNO055IMU imu = null;

    /**
     * Method to initialize standard Hardware interfaces
     **/
    void init(HardwareMap ahwMap) {
        // Save reference to Hardware map

        //------------------------------------------------------------------------------------------
        //Define and Initialize Motors
        FL = ahwMap.get(DcMotor.class, "FL");
        FR = ahwMap.get(DcMotor.class, "FR");
        BL = ahwMap.get(DcMotor.class, "BL");
        BR = ahwMap.get(DcMotor.class, "BR");
        liftMotor = ahwMap.get(DcMotor.class, "liftMotor");
        tensionMotor = ahwMap.get(DcMotor.class, "tensionMotor");
        //Current Motor Count - 6


        //------------------------------------------------------------------------------------------
        clampL = ahwMap.get(Servo.class, "clampL");
        clampR = ahwMap.get(Servo.class, "clampR");
        foundationL = ahwMap.get(Servo.class, "foundationL");
        foundationR = ahwMap.get(Servo.class, "foundationR");
        //Current Servo Count - 4

        //------------------------------------------------------------------------------------------
        // Define webcam
        //camera = ahwMap.get(WebcamName.class, "Webcam 1");

        //------------------------------------------------------------------------------------------
        // Define and initialize ALL installed sensors.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        //Imu
        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        sensorFront = ahwMap.get(DistanceSensor.class, "sensorFront");

        //------------------------------------------------------------------------------------------
        //Hardware moves

        //Reverse the right motors to make code operation easier
        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        tensionMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set all motors to zero power to prevent unintended movement
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        liftMotor.setPower(0);
        tensionMotor.setPower(0);
    }

}