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

public class ArmControlThreadTest extends TestCase {
    testMotor motor1 = new testMotor();
    testServo servo1 = new testServo();
    testServo servo2 = new testServo();
    testServo servo3 = new testServo();
    testServo servo4 = new testServo();
    testDistance testDistance1 = new testDistance();
    testController testController1 = new testController();
    public void testArmMovement()
    {
        ArmControlThread armControlThread = new ArmControlThread(motor1,servo1,servo2,servo3,servo4,testDistance1,10,testController1);
        armControlThread.run();
    }
}