/*
 * Copyright (c) 2019.  [Bradley Abelman]
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

import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.util.FastMath;
import org.jetbrains.annotations.NotNull;

public class testAccel
{
    public static void main(String[] args)
    {

        double speed = 0;
        int i = 0;
        for (int j = 0; j < 1; j++)
        {
            i = (int) FastMath.round(FastMath.random() + 1);
            if(i%2 == 0)
            {
                i = -1;
            }
            else
                {
                    i = 1;
                }


            double powerRn = 0.000000001;

            System.out.println(powerRn);
            speedChange(powerRn,1,DirectAccel.accel,1);

            speedChange(powerRn,-1,DirectAccel.decel,1);

            System.out.println();

            powerRn = -0.01; //anything lower is 0 affective due to the isZero method


            System.out.println(powerRn);
            speedChange(powerRn,-1,DirectAccel.accel,1);

            speedChange(powerRn,1,DirectAccel.decel,1);

            System.out.println();

            powerRn = 0.00000000;


            System.out.println(powerRn);
            speedChange(powerRn,-1,DirectAccel.accel,1);
            speedChange(powerRn,1,DirectAccel.accel,1);

            speedChange(powerRn,   1,DirectAccel.decel,1);
            speedChange(powerRn,   -1,DirectAccel.decel,1);

            System.out.println();
        }
    }


    /**
     * @param motorFront
     */
    private static void speedChange(@NotNull double motorFront, double powerSign, DirectAccel directAccel, double accelFactor)
    {
        double oldPower = FastMath.abs(motorFront);

        if (oldPower < 0.01 && directAccel.equals(DirectAccel.accel))
        {
            oldPower = FastMath.copySign(0.01, powerSign);
        }

        oldPower = FastMath.abs(oldPower);
        double stepPower = FastMath.nextAfter(oldPower,accelType(directAccel));

        stepPower -= oldPower;

        stepPower *= accelFactor * FastMath.pow(10,15);

        double newPower = oldPower + stepPower;

        newPower = FastMath.copySign(newPower,powerSign);

        newPower = isZero(newPower);

        newPower = Range.clip(newPower,-1.0,1.0);

        System.out.println(newPower);
    }

    private enum DirectAccel {accel,decel}

    private static double accelType(DirectAccel directAccel)
    {
        if(directAccel.equals(DirectAccel.accel))
        {
            return Double.POSITIVE_INFINITY;
        }
        return Double.NEGATIVE_INFINITY;
    }


    private static double isZero(double power)
    {
        if(power < 0.01 && power > -0.01)
        {
            return 0;
        }
        return power;
    }
}