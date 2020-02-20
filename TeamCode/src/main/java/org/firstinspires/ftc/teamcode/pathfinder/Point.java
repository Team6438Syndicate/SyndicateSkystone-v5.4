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

import org.apache.commons.math3.util.FastMath;

public class Point {
    private double x;
    private double y;

    /**
     * @param x
     * @param y
     */
    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * @return
     */
    public double getX() {
        return x;
    }

    public void setX(final double x) {
        this.x = x;
    }

    /**
     * @return
     */
    public double getY() {
        return y;
    }

    public void setY(final double y) {
        this.y = y;
    }

    /**
     * @return
     */
    public double[] returnCartesian() {
        return new double[]{x,y};
    }

    @SuppressWarnings("SuspiciousNameCombination")
    public void rotatePoint(double rotation)
    {
        double newX =  (getX() * FastMath.cos(FastMath.toRadians(rotation)) - getY() * FastMath.sin(FastMath.toRadians(rotation)));
        double newY =  (getX() * FastMath.sin(FastMath.toRadians(rotation)) + getY() * FastMath.cos(FastMath.toRadians(rotation)));

        if(FastMath.abs(newX) < 0.001)
        {
            newX = 0;
        }
        if(FastMath.abs(newY) < 0.001)
        {
            newY = 0;
        }
        setX(newX);
        setY(newY);
    }

    public void appendPoint(double marginX, double marginY)
    {
        setX(getX() + marginX);
        setY(getY() + marginY);
    }
}
