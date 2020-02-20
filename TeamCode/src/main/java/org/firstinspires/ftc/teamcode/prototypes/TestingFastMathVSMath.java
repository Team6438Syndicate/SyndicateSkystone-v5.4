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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.apache.commons.math3.util.FastMath;

import java.util.ArrayList;

@Deprecated
@Disabled
public class TestingFastMathVSMath {
    public static void main(String[] args) {
        int n = 10000;

        double[] x = new double[n];
        double[] y = new double[n];

        for (int i = 0; i < n; i++) {
            x[i] = FastMath.random();
            y[i] = FastMath.random();

        }

        long fo = System.currentTimeMillis();
        for (int i = 0; i < n; i++) {
            // System.out.println(FastMath.sin(3.0 / 4.0));
            calcArmAngles(x[i], y[i]);

        }
        long end = System.currentTimeMillis();
        System.out.println("Time: " + (end - fo) + "," + ((double) (end - fo) / (double) n));
        /* fo = System.currentTimeMillis();

        for (int i = 0; i < n; i++)
        {
           // System.out.println(Math.sin(3.0 / 4.0));
            calcArmAngles(x[i],y[i]);

        }
        System.out.println("Time: " + (System.currentTimeMillis() - fo));*/

    }

    protected static ArrayList<Double> calcArmAngles(double tX, double tY) {

        float FL;// = Team6438ChassisHardwareMapCurrent.FOREARM_LENGTH;
        float SL;// = Team6438ChassisHardwareMapCurrent.SHOULDER_LENGTH;

        ArrayList<Double> armAngles = new ArrayList<>();

        double tD = (FastMath.hypot(tX, tY));

        //double elbowAngle = (float) FastMath.acos(((tD * tD - SL * SL - FL * FL) / (- 2f * SL * FL)));
        //armAngles.add(elbowAngle);

        //double wristAngle = (FastMath.asin(FL * FastMath.sin(elbowAngle) / tD));
        //armAngles.add(wristAngle);

        //double shoulderAngle = (FastMath.atan(tX / tY) + 180 - elbowAngle - wristAngle);
        //armAngles.add(shoulderAngle);

        return armAngles;
    }
}
