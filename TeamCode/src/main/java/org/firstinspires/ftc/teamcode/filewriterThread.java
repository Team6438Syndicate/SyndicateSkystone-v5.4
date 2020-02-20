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

import com.qualcomm.robotcore.util.ElapsedTime;

import org.jetbrains.annotations.Contract;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class filewriterThread implements Runnable {

    private String opMode;
    FileWriter fw;
    ElapsedTime time;

    protected filewriterThread(ElapsedTime time, String opMode)
    {
        this.time = time;
        this.opMode = opMode;
    }

    @Override
    public void run()
    {
        time.reset();
        File file = new File(Environment.getExternalStorageDirectory().getPath() + "/Download/testLogs.txt");
        int counter = readTestNumber(file);

        try
        {
            fw = new FileWriter(file,true); //the true will append the new data
            fw.write(counter);
            write("Running " + opMode + " OpMode");
        }
        catch (IOException ignored)
        {
        }
    }

    public void write(String info)
    {
        if (fw != null)
        {
            try
            {
                fw.write(info + "\tTime: " + time.milliseconds() + "\r");
            }
            catch(IOException ignored)
            {}
        }

    }

    protected void doStop()
    {
        endTest();
    }

    private void endTest()
    {
        try
        {
            fw.write("END OF TEST\tTime: " + time.milliseconds() + "\r\n");
        }
        catch (IOException ignored)
        {}
    }

    private int readTestNumber(File file)
    {

        List<Integer> testNums = new ArrayList<>();
        try (FileReader fileReader = new FileReader(file)) {

            BufferedReader bufferedReader = new BufferedReader(fileReader);
            String line;

            while ((line = bufferedReader.readLine()) != null && ! line.equals(""))
            {
                String[] split = line.split("\n[0-9]*");

                for(int i = 0; i < split.length; i++)
                {
                    testNums.add(Integer.valueOf(split[0]));
                }
            }

        } catch (IOException e) {
            return 0;
        }

        int maxTestNum = testNums.get(0);
        for (int i = 0; i < testNums.size(); i++)
        {
            if (maxTestNum < testNums.get(i))
            {
                maxTestNum = testNums.get(i);
            }
        }

        return maxTestNum;
    }
}
