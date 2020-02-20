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

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Team6438ChassisHardwareMapCurrent;
import org.jetbrains.annotations.Contract;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Objects;

public class PositionFileWriteOutThread implements Runnable {
    private OpMode opMode;
    private Team6438ChassisHardwareMapCurrent hardwareMapCurrent;
    private boolean extendedPrintout;

    public PositionFileWriteOutThread(OpMode opMode, Team6438ChassisHardwareMapCurrent hardwareMapCurrent, final boolean extendedPrintout)
    {
        this.opMode = opMode;
        this.hardwareMapCurrent = hardwareMapCurrent;
        this.extendedPrintout = extendedPrintout;
        hardwareMapCurrent.imu.startAccelerationIntegration(new Position(), new Velocity(), 10);

    }

    private boolean doStop;

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
        File file = new File(Environment.getExternalStorageDirectory().getPath() + "/Download/Logs.txt");
        FileWriter fw = null;
        BufferedWriter writer = null;
        PrintWriter printWriter = null;
        try
        {
            fw = new FileWriter(file);
            fw.write(String.valueOf(opMode));
           writer = new BufferedWriter( fw);
           printWriter=new PrintWriter(writer);
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        ElapsedTime elapsedTime = new  ElapsedTime();
        elapsedTime.reset();
        while (keepRunning())
        {
            if(elapsedTime.milliseconds()%100 == 0)
            {
                if (fw != null)
                {
                    if (printWriter != null)
                    {
                        printWriter.println("<<<*<");

                        printWriter.println("TimeStamp: " + elapsedTime.milliseconds());
                        printWriter.println(hardwareMapCurrent.imu.getPosition().toUnit(DistanceUnit.INCH).toString());

                        printWriter.println(hardwareMapCurrent.imu.getVelocity().toString());
                        printWriter.println(hardwareMapCurrent.imu.getAcceleration());
                        printWriter.println(hardwareMapCurrent.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).toString());
                        printWriter.println(hardwareMapCurrent.imu.getAngularVelocity());
                        printWriter.println(">*>");
                        if (extendedPrintout)
                        {
                            printWriter.println("FL: " + hardwareMapCurrent.FL.getCurrentPosition() +" "+ hardwareMapCurrent.FL.getTargetPosition() +" "+ hardwareMapCurrent.FL.getPower());
                            printWriter.println("FR: " + hardwareMapCurrent.FR.getCurrentPosition() +" "+ hardwareMapCurrent.FR.getTargetPosition() +" "+ hardwareMapCurrent.FR.getPower());
                            printWriter.println("BL: " + hardwareMapCurrent.BL.getCurrentPosition() +" "+ hardwareMapCurrent.BL.getTargetPosition() +" "+ hardwareMapCurrent.BL.getPower());
                            printWriter.println("BR: " + hardwareMapCurrent.BR.getCurrentPosition() +" "+ hardwareMapCurrent.BR.getTargetPosition() +" "+ hardwareMapCurrent.BR.getPower());
                            printWriter.println("LIFT: " + hardwareMapCurrent.liftMotor.getCurrentPosition() +" "+ hardwareMapCurrent.liftMotor.getTargetPosition() +" "+ hardwareMapCurrent.liftMotor.getPower());
                            printWriter.println("FoundationL: " + hardwareMapCurrent.foundationL.getPosition());
                            printWriter.println("FoundationR: " + hardwareMapCurrent.foundationR.getPosition());
                            printWriter.println("ClampL: " + hardwareMapCurrent.clampL.getPosition());
                            printWriter.println("ClampR: " + hardwareMapCurrent.clampR.getPosition());
                            printWriter.println(">>");
                        }


                    }


                }
            }
        }
        try
        {
            Objects.requireNonNull(fw).close();

            Objects.requireNonNull(writer).close();
            printWriter.close();

        }
        catch (IOException e)
        {
            e.printStackTrace();
        }

    }
   public synchronized void doStop()
    {

        this.doStop = true;


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


}
