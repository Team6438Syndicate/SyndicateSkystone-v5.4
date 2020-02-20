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


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
import static org.firstinspires.ftc.teamcode.Team6438ChassisHardwareMapCurrent.bridgeRotY;
import static org.firstinspires.ftc.teamcode.Team6438ChassisHardwareMapCurrent.bridgeRotZ;
import static org.firstinspires.ftc.teamcode.Team6438ChassisHardwareMapCurrent.bridgeX;
import static org.firstinspires.ftc.teamcode.Team6438ChassisHardwareMapCurrent.bridgeY;
import static org.firstinspires.ftc.teamcode.Team6438ChassisHardwareMapCurrent.bridgeZ;
import static org.firstinspires.ftc.teamcode.Team6438ChassisHardwareMapCurrent.halfField;
import static org.firstinspires.ftc.teamcode.Team6438ChassisHardwareMapCurrent.mmPerInch;
import static org.firstinspires.ftc.teamcode.Team6438ChassisHardwareMapCurrent.mmTargetHeight;
import static org.firstinspires.ftc.teamcode.Team6438ChassisHardwareMapCurrent.quadField;

/*
 * test for the map of the field
 *
 * Initially Created 3/21/19
 * Author: Matthew Kaboolian
 *
 */
public class CoordinateMapMaker {
    private static final String VUFORIA_KEY = Team6438ChassisHardwareMapCurrent.VUFORIA_KEY;
    private VuforiaLocalizer.CameraDirection CAMERA_CHOICE;
    private CameraName cameraName;
    private boolean PHONE_IS_PORTRAIT;
    private float phoneXRotate;
    private float phoneYRotate;
    private float phoneZRotate;
    private  VuforiaLocalizer vuforia;
    // It is centered (left to right), but forward of the middle of the robot, and above ground level.
    private float CAMERA_FORWARD_DISPLACEMENT;
    private float CAMERA_VERTICAL_DISPLACEMENT;
    private float CAMERA_LEFT_DISPLACEMENT;
    private VuforiaTrackables targetsSkyStone = null;
    private OpenGLMatrix lastLocation = null;
    private String name;

    public CoordinateMapMaker(@NotNull HardwareMap robot, VuforiaLocalizer.CameraDirection CAMERA_CHOICE, String name, boolean isPortrait, float angleX, float angleY, float angleZ, float forwDis, float vertDis, float leftDis) {
        this.name = name;
        this.CAMERA_CHOICE = CAMERA_CHOICE;
        this.cameraName = robot.get(WebcamName.class, name);
        this.PHONE_IS_PORTRAIT = isPortrait;
        this.phoneXRotate = angleX;
        this.phoneYRotate = angleY;
        this.phoneZRotate = angleZ;
        CAMERA_FORWARD_DISPLACEMENT = forwDis * mmPerInch;
        CAMERA_VERTICAL_DISPLACEMENT = vertDis * mmPerInch;
        CAMERA_LEFT_DISPLACEMENT = leftDis * mmPerInch;

        setOpenGLMatrixNavTarg(robot);
        birthVuforia();

    }

    public CoordinateMapMaker(@NotNull HardwareMap robot, VuforiaLocalizer.CameraDirection CAMERA_CHOICE, boolean isPortrait, float angleX, float angleY, float angleZ, float forwDis, float vertDis, float leftDis) {

        this.CAMERA_CHOICE = CAMERA_CHOICE;

        this.PHONE_IS_PORTRAIT = isPortrait;
        this.phoneXRotate = angleX;
        this.phoneYRotate = angleY;
        this.phoneZRotate = angleZ;
        CAMERA_FORWARD_DISPLACEMENT = 5 * mmPerInch;
        CAMERA_VERTICAL_DISPLACEMENT = 0 * mmPerInch;
        CAMERA_LEFT_DISPLACEMENT = 5 * mmPerInch;

        setOpenGLMatrixNavTarg(robot);
        birthVuforia();
    }
    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT

    private void setOpenGLMatrixNavTarg(HardwareMap robot) {

        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        int cameraMonitorViewId = robot.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        if (cameraName != null) {
            parameters.cameraName = cameraName;
        }

        /*if(name != null && name.contains("Web"))
        {
            int cameraMonitorViewId = robot.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.appContext.getPackageName());
           parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        }
        */


        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = this.CAMERA_CHOICE;
        parameters.useExtendedTracking = true;
        //  Instantiate the Vuforia engine
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
System.out.println("!X!");


        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<>(targetsSkyStone);

        /*
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(- bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(- bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, - bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(- bridgeX, - bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, - bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, - bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, - halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(- quadField, - halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(- halfField, - quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(- halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(- quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, - 90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, - quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, - 90)));

//
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = - 90;
        } else if (CAMERA_CHOICE == FRONT) {
            phoneYRotate = 90;
        }


        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }


        // Next, translate the camera lens to where it is on the robot.
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /*
           Let all the trackable listeners know where the phone is.

         */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

    this.targetsSkyStone = targetsSkyStone;
    }

    /**
     * activate vuforia
     */
    private void birthVuforia() {
        this.targetsSkyStone.activate();
    }

    /**
     * Kill vuforia call able on @stop or when the timer expires
     */
    void killVuforia() {
        targetsSkyStone.deactivate();
    }

    public ArrayList<ArrayList<Float>> runVuforia(VuforiaTrackables vuforiaTrackables) {
        ArrayList<ArrayList<Float>> returns = new ArrayList<>();
        ArrayList<Float> returnCoords = new ArrayList<>();
        ArrayList<Float> orientations = new ArrayList<>();

        // check all the trackable targets to see which one (if any) is visible.
        boolean targetVisible = false;
        for (VuforiaTrackable trackable : vuforiaTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();

            returnCoords.add(translation.get(0) / mmPerInch);
            returnCoords.add(translation.get(1) / mmPerInch);
            returnCoords.add(translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);

            orientations.add(rotation.firstAngle);
            orientations.add(rotation.secondAngle);
            orientations.add(rotation.thirdAngle);
        }
        returns.add(returnCoords);
        returns.add(orientations);

        return returns;

    }

    public VuforiaTrackables getTargetsSkyStone() {
        return targetsSkyStone;
    }

    public void setTargetsSkyStone(VuforiaTrackables targetsSkyStone) {
        this.targetsSkyStone = targetsSkyStone;
    }
}







