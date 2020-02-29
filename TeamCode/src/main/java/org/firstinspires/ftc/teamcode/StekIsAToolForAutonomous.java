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

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
@Disabled
@Autonomous(name = "Make the locations...Stek you know", group = "Team 6438 Driver Controlled")
public class StekIsAToolForAutonomous extends LinearOpMode {
    private CoordinateMapMaker vuforiaCamera2;

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException stuff
     */
    @Override
    public void runOpMode() throws InterruptedException {



        File file = new File(Environment.getExternalStorageDirectory().getPath() + "/Download/locations.txt");

        File fileObs = new File(Environment.getExternalStorageDirectory().getPath() + "/Download/Obslocations.txt");

        int counter = 0;

        int counterObs = 0;
        try
        {
            FileWriter fw = new FileWriter(file,true); //the true will append the new data
            FileWriter fwObs = new FileWriter(file,true); //the true will append the new data

            createVuforia();

            waitForStart();


            while (opModeIsActive()) {
                if (gamepad1.a) {
                    checkVuforia();
                    String a = new StringBuilder()
                            .append(counter).append(",")
                            .append(validatedCoordsAngles.get(0).get(0))
                            .append(",")
                            .append(validatedCoordsAngles.get(0).get(1))
                            .append(",")
                            .append(validatedCoordsAngles.get(0).get(2))
                            .append(",")
                            .append(validatedCoordsAngles.get(1).get(0))
                            .append(",")
                            .append(validatedCoordsAngles.get(1).get(1))
                            .append(",")
                            .append(validatedCoordsAngles.get(1).get(2))
                            .append("\r")
                            .toString();
                    fw.write(a);


                    telemetry.addData("Location has been added", counter + 1);




                    telemetry.update();
                    ++counter;

                }
                if (gamepad1.b) {
                    checkVuforia();
                    String a = new StringBuilder()
                            .append(counter).append(",")
                            .append(validatedCoordsAngles.get(0).get(0))
                            .append(",")
                            .append(validatedCoordsAngles.get(0).get(1))
                            .append(",")
                            .append(validatedCoordsAngles.get(0).get(2))
                            .append(",")
                            .append(validatedCoordsAngles.get(1).get(0))
                            .append(",")
                            .append(validatedCoordsAngles.get(1).get(1))
                            .append(",")
                            .append(validatedCoordsAngles.get(1).get(2))
                            .append("\r")
                            .toString();
                    fwObs.write(a);


                    telemetry.addData("Location has been added", counterObs + 1);




                    telemetry.update();
                    ++counterObs;

                }

                if (isStopRequested()) {
                    fw.close();
                    fwObs.close();
                }
            }


        } catch (Exception e) {
            //System.out.println(Environment.getDataDirectory());

            e.printStackTrace();
            telemetry.addData("File Error", true);
            telemetry.addData("Stack", e);
          //  telemetry.addData("Trace", e.getStackTrace().toString());

            for (StackTraceElement ex: e.getStackTrace())
            {
             telemetry.addData("trace", ex.toString());
            }
            telemetry.update();
            wait(15000);
        }

    }

    ArrayList<ArrayList<Float>> validatedCoordsAngles = new ArrayList<>(); //Brad here is your code
    private CoordinateMapMaker vuforiaCamera1;
    private ArrayList<ArrayList<Float>> arrayList1 = new ArrayList<>();

    /**
     * Create the cameras
     */
    protected void createVuforia() {

        vuforiaCamera1 = new CoordinateMapMaker(hardwareMap, VuforiaLocalizer.CameraDirection.BACK, "Webcam 1", false, 0, 0, 0, 0, 0, 0);
        //vuforiaCamera2 = new CoordinateMapMaker(hardwareMap, VuforiaLocalizer.CameraDirection.BACK, "Webcam 2", false, 0, 0, 0, 0, 0, 0);

    }

    /**
     * Check the vuforia obkects for tracked objects
     */
    protected void checkVuforia() {



        arrayList1 = vuforiaCamera1.runVuforia(vuforiaCamera1.getTargetsSkyStone());


        validatedCoordsAngles = arrayList1;


    }
}
