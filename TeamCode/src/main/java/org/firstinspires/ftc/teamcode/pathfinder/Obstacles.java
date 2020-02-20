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

package org.firstinspires.ftc.teamcode.pathfinder;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

public class Obstacles
{
          private ArrayList<Shape> obsShapes = new ArrayList<>();
     public Obstacles()
     {
         Shape targ1 = new Shape(0,0,2,2,0,0, DistanceUnit.INCH);
         System.out.println(targ1.toString());

         Shape targ2 = new Shape(1,1,2,2,0,0,DistanceUnit.INCH);
         System.out.println(targ2.toString());

         Shape targ3 = new Shape(-1,-1,2,2,0,0,DistanceUnit.INCH);
         System.out.println(targ3.toString());

         Shape targ4 = new Shape(1,-1,2,2,0,0,DistanceUnit.INCH);
         System.out.println(targ4.toString());

         Shape targ5 = new Shape(-1,1,2,2,0,0,DistanceUnit.INCH);
         System.out.println(targ5.toString());

         obsShapes.add(targ1);
         obsShapes.add(targ2);
         obsShapes.add(targ3);
         obsShapes.add(targ4);
         obsShapes.add(targ5);

     }







}
