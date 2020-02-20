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

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class testController extends Gamepad
{
    public testController()
    {
        a = true;
    }

    private float[] valuesJoy = {-1,-.75f,-.5f,-.25f, 0.0f,.25f,.50f,.75f,1.0f};


    public void setLStickY(int lY)
    {
      left_stick_y = valuesJoy[lY];
    }

     void setLStickY(float value)
    {
        left_stick_y = Range.clip(value,-1.0f,1.0f);
    }

    public void setRStickY(int rY)
    {
        right_stick_y = valuesJoy[rY];
    }

    @Override
    public String toString()
    {
        return "left stick: " + left_stick_y + " " + "right stick: " + right_stick_y;
    }

    public boolean a()
    {
        a = true;
        return true;
    }

}
