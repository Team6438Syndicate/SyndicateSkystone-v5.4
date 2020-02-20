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

package org.firstinspires.ftc.teamcode.pathfinder;

import junit.framework.TestCase;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;

public class PathFinderTest extends TestCase {

    File file2 = new File("C:\\test" + "\\locations.txt");
    File file1 = new File("C:\\test" + "\\Obslocations.txt");
    ArrayList<Float> here = new ArrayList<>();

    public void testRun() {
        here.add(0f);
        here.add(0f);
        here.add(0f);
        here.add(0f);
        here.add(0f);
        here.add(0f);

        PathFinder pathFinder = new PathFinder(here, 0, file1, file2);

        PathFinder.Path path = pathFinder.pathFinding();

        if (path == null) {
            System.out.println("All possible paths are unsafe");
            // TODO: 11/22/2019 add safe stop stuff here
        }
        else {
            System.out.println(path.toString());
            System.out.println(Arrays.toString(pathFinder.runtimePathCheck(path)));
        }
    }
}