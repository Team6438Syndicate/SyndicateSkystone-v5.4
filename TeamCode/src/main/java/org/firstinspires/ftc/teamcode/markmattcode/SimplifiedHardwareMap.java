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
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

public class SimplifiedHardwareMap
{
    //Vuforia Variables
    public static final String VUFORIA_KEY = "ATEEWHn/////AAABmXzvuqxXZkYkr3AeTQT4Qg0P3tudpoBP/Rp2Xyw3zNlZYk+ZI5Jp/yo8TDf62o+UjdBvoe0LP5nNDqFESCtSImOG2WRuMkoESAyhSVzMU0hY53dWb4l0s7mCe+xqqT8i0r9pPdav7N7RiGHG7WYoIBXrQeyz+NEq8TLYTTCXmZMFgPeEU30Nb+t4JikoNMr0X0Ej6y1vG+7EX3O9KI8RXoPYbBmPzvX5uVvWBNg2J0g0SBiZUXa8pQOCxi0QyHyNUiwvV5WKnM2jncg+eI7im5s+k4yn6Xjaeecg6q9IT45YNvbhV4PM/LbwGQTKBf0AOCM/qL7tz7evypWw5uK15BayqAitBLy7Sr0SvIjYMjPg";
    public static final double     COUNTS_PER_MOTOR_REV_DRIVE    = 383.6 ;    // eg: TETRIX Motor Encoder
    public static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    public static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public final int        driveCPI        =         (int) ((COUNTS_PER_MOTOR_REV_DRIVE * DRIVE_GEAR_REDUCTION) /
                                                            (WHEEL_DIAMETER_INCHES * FastMath.PI));
    //-------------------------------
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

    //------
    static final double countsRulerMotor = 145.6;
    static final double DGR_Ruler = 1;
    static final double RulerWheelDiam = 2;
    public static final int rulerCPI = (int) ((countsRulerMotor * DGR_Ruler) / (RulerWheelDiam * FastMath.PI));

    //Motor  Declaration
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;
    public DcMotor liftMotor = null;
    //public DcMotor tensionMotor = null;
    public DcMotor rulerMotor = null;

    //Servo Declaration
    public  Servo clampL = null;
    public  Servo clampR = null;
    public  Servo foundationL = null;
    public  Servo foundationR = null;
    public  Servo capstone = null;

    //Webcam mapping
    public WebcamName camera = null;

    //Sensor Mapping
    public Rev2mDistanceSensor frontSensor;
    public BNO055IMU imu = null;

    /**
     * Method to initialize standard Hardware interfaces
     **/
   public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map

        //------------------------------------------------------------------------------------------
        //Define and Initialize Motors
        FL = ahwMap.get(DcMotor.class, "FL");
        FR = ahwMap.get(DcMotor.class, "FR");
        BL = ahwMap.get(DcMotor.class, "BL");
        BR = ahwMap.get(DcMotor.class, "BR");
        liftMotor = ahwMap.get(DcMotor.class, "liftMotor");
        //tensionMotor = ahwMap.get(DcMotor.class, "tensionMotor");
        rulerMotor = ahwMap.get(DcMotor.class, "rulerMotor");
        //Current Motor Count - 6


        //------------------------------------------------------------------------------------------
        clampL = ahwMap.get(Servo.class, "clampL");
        clampR = ahwMap.get(Servo.class, "clampR");
        foundationL = ahwMap.get(Servo.class, "foundationL");
        foundationR = ahwMap.get(Servo.class, "foundationR");
        capstone = ahwMap.get(Servo.class,"capstone");
        //Current Servo Count - 5

        //------------------------------------------------------------------------------------------
        // Define webcam
        //camera = ahwMap.get(WebcamName.class, "Webcam 1");

        //------------------------------------------------------------------------------------------


        //Imu
        imu = ahwMap.get(BNO055IMU.class, "imu");

        frontSensor = ahwMap.get(Rev2mDistanceSensor.class, "sensorFront");

        // Define and initialize ALL installed sensors.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu.initialize(parameters);

        //------------------------------------------------------------------------------------------
        //Hardware moves

        //Reverse the right motors to make code operation easier
        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rulerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //tensionMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set all motors to zero power to prevent unintended movement
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        liftMotor.setPower(0);
        rulerMotor.setPower(0);
        //tensionMotor.setPower(0);
    }

}