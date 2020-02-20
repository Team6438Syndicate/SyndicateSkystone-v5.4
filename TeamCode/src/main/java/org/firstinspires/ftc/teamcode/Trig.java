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

import org.apache.commons.math3.util.FastMath;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;

import static org.apache.commons.math3.util.FastMath.*;

class Trig
{

    private static final int precision = 100; // gradations per degree, adjust to suit

    private static final int modulus = 90 * precision;

    private static final ArrayList<Float> sinTest = new ArrayList<Float>();
    static
    {


        // a static initializer fills the table
        // in this implementation, units are in degrees

        for (int i = 0; i < modulus; i++)
        {
            double angle = (i * PI) / (2 * modulus);


            double number = FastMath.sin(angle);

            sinTest.add(round(number));

        }
    }

    static float round(double value)
    {
        BigDecimal bd = new BigDecimal(Double.toString(value));
        bd = bd.setScale(4, RoundingMode.HALF_UP);
        return (float) bd.doubleValue();
    }

    // Private function for table lookup
    private static float sinLookup(int a)
    {
        return a >= 0 ? sinTest.get(a % modulus) : -sinTest.get(-a % modulus);
    }

    // Private function for table lookup
    private static float arcSinLookup(float a)
    {
        float num1 = sinTest.indexOf(round(a));


        if (num1 == -1)
        {

            num1 = arcSinLookup(a + 0.0001f);
        }
        else
        {
            num1 = num1 / 100.0f;
        }


        return num1;
    }

    // These are your working functions:
    static float sin(float a)
    {
        return sinLookup((int) (a * precision + 0.5f));
    }

    static float cos(float a)
    {
        return sinLookup((int) ((90 - a) * precision + 0.5f));
    }

    static float arcSin(float a)
    {
        return arcSinLookup(a);
    }

    static float arcCos(float a)
    {
        return a >= 0 ? -arcSinLookup(a) + 90 : arcSinLookup(-a) + 90;

    }

    static float arcTan(float a)
    {
        float test = arcSinLookup(round(a / (sqrt(1 + a * a))));
        return test;
    }



    static double lawOfCosines(double hyp, double a, double b) {
        double angle = acos((hyp*hyp - a*a - b*b) / (- 2.0f * a * b));
        return  angle;

    }

    static double ramaujansFormula(float a, float b)
    {
        double top = a - b;
        double bottom = a + b;
        double h = pow((top), 2) / pow((bottom), 2);


        return (PI / 4) * bottom * (1 + ((3 * h) / (10 + sqrt(4 - 3 * h))));
    }
}
