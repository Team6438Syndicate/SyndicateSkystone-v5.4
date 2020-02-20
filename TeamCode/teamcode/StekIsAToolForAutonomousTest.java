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

import junit.framework.TestCase;

import org.apache.commons.math3.util.FastMath;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class StekIsAToolForAutonomousTest extends TestCase {

    public void testRunOpMode()
    {
        File file = new File("C:\\test" + "\\locations.txt");

        try {


            FileWriter fw = new FileWriter(file,true); //the true will append the new data


            for (int i = 0; i < 50; i++){
                if (true) {



                    fw.write(new StringBuilder()
                            .append((i+1) * FastMath.random()*2)
                            .append(" X, ")
                            .append((i+1) * FastMath.random()*2)
                            .append(" Y, ")
                            .append((i+1) * FastMath.random()*2)
                            .append(" Z, ")
                            .append((i+1) * FastMath.random()*2)
                            .append(" XTheta, ")
                            .append((i+1) * FastMath.random()*2)
                            .append(" YTheta, ")
                            .append((i+1) * FastMath.random()*2)
                            .append(" ZTheta")
                            .toString());//appends the string to the file
                    fw.append("\n\n");
                }

            }
            fw.close();



        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}