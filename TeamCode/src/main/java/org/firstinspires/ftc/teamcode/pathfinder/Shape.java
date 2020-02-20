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
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;

public class Shape {

    private final double centerX;
    private final double centerY;
    private final double rotation;
    private final double marginal;
    private final DistanceUnit distanceUnit;
    private double endLength;
    private double endWidth;

    private Point center;
    private Point vertexUpperLeft;
    private Point vertexUpperRight;
    private Point vertexLowerLeft;
    private Point vertexLowerRight;

    @SuppressWarnings("SuspiciousNameCombination")
    public Shape(double centerX, double centerY, double endLength, double endWidth, double rotation, double marginal, DistanceUnit distanceUnit) {
        this.centerX = centerX;
        this.centerY = centerY;
        this.endLength = endLength;

        this.endWidth = endWidth;

        this.rotation = rotation;


        this.marginal = marginal;
        this.distanceUnit = distanceUnit;

        center = new Point(centerX, centerY);
        formatShapeAndArea();

        double halfLength = endLength / 2.0;
        double halfWidth = endWidth / 2.0;

        vertexUpperLeft = new Point(- halfLength, halfWidth);
        vertexLowerLeft = new Point(- halfLength, - halfWidth);
        vertexUpperRight = new Point(halfLength, + halfWidth);
        vertexLowerRight = new Point(halfLength, - halfWidth);


        rotateShape();
        translateShape();
    }





    private void setEndLength(final double endLength) {
        this.endLength = endLength;
    }



    private void setEndWidth(final double endWidth) {
        this.endWidth = endWidth;
    }

    private void formatShapeAndArea() {
        double newEndLength;

        newEndLength = endLength + marginal;

        setEndLength(newEndLength);


        double newEndWidth;

        newEndWidth = endWidth + marginal;

        setEndWidth(newEndWidth);
    }

    private void rotateShape() {

        vertexLowerLeft.rotatePoint(rotation);
        vertexUpperRight.rotatePoint(rotation);
        vertexLowerRight.rotatePoint(rotation);
        vertexUpperLeft.rotatePoint(rotation);

    }

    private void translateShape() {


        vertexUpperLeft.appendPoint(centerX,centerY);
        vertexLowerRight.appendPoint(centerX,centerY);
        vertexLowerLeft.appendPoint(centerX,centerY);
        vertexUpperRight.appendPoint(centerX,centerY);

    }





    @NotNull
    @Override
    public String toString() {
        double[][][] verticies =
                {
                        {vertexUpperLeft.returnCartesian(),vertexUpperRight.returnCartesian()}
                        ,
                        {vertexLowerLeft.returnCartesian(),vertexLowerRight.returnCartesian()}
                };

        String returning = "All units are in: " +
                distanceUnit.toString() +
                "\n" +
                "Center @: " +
                Arrays.toString(center.returnCartesian()) +
                "\n" +
                "Vertices @ " +
                Arrays.deepToString(verticies) +
                "\n" +
                "Margin = " +
                marginal +
                "\n";
        return super.toString() + "\n" + returning;
    }

    private enum TranslationDirection {
        upperLeft, upperRight, lowerLeft, lowerRight, up, down, left, right, holdPos
    }
}



