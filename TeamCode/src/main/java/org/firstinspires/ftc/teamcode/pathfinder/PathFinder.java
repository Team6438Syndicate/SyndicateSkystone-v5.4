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

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static org.apache.commons.math3.util.FastMath.abs;
import static org.apache.commons.math3.util.FastMath.atan;
import static org.apache.commons.math3.util.FastMath.copySign;
import static org.apache.commons.math3.util.FastMath.hypot;
import static org.apache.commons.math3.util.FastMath.pow;
import static org.apache.commons.math3.util.FastMath.sqrt;

public class PathFinder
{
    public List<positionInformationRecord> targetCoords;
    public List<positionInformationRecord> obstacleCoords;
    private ArrayList<Float> robotPosition;
    private int targetIndex;
    private double range;

    public PathFinder(ArrayList<Float> robotPosition, int targetIndex, double range) {
        this.robotPosition = robotPosition;

        this.targetIndex = targetIndex;
        this.range = range;

        obstacleCoords = setObstacleCoords();
        targetCoords = setTargetCoords();
    }

    PathFinder(ArrayList<Float> robotPosition, int targetIndex, File file1, File file2) {
        this.robotPosition = robotPosition;

        this.targetIndex = targetIndex;

        obstacleCoords = setTargetCoords(file1);
        targetCoords = setTargetCoords(file2);
    }


    private static float tangent(@NotNull Path path, @NotNull positionInformationRecord obstacles, positionInformationRecord target, ArrayList<Float> robotPosition) {

        // TODO: 11/22/2019 check if the given coords correspond to the actual position on the field
        float p = obstacles.getX();
        float q = obstacles.getY();

        switch (path) {
            case Direct:

                float deltaY = target.getY() - robotPosition.get(1);
                float deltaX = target.getX() - robotPosition.get(0);

                //System.out.println(slope);


                return deltaY / deltaX;

            case Curved:

                float a = target.getX() - robotPosition.get(0);

                //System.out.println(tangentSlope);

                return q / (p - a);

            default:
                throw new IllegalStateException("Illegal Path Type: " + path);
        }
    }

    private static float deltaY(positionInformationRecord target, ArrayList<Float> robotPosition) {
        return target.getY() - robotPosition.get(1);
    }

    private static float deltaX(positionInformationRecord target, ArrayList<Float> robotPosition) {
        return target.getX() - robotPosition.get(0);
    }

    private static float findDistance(float p, float q, float x, float y) {
        return (float) hypot(x - p, y - q);
    }

    private static boolean checkDomain(positionInformationRecord obstacles, positionInformationRecord target, ArrayList<Float> robotPosition) {
        if (target.getX() > robotPosition.get(0)) {
            return ! (obstacles.getX() < robotPosition.get(0) - 9) && ! (obstacles.getX() > target.getX() + 9);

        }
        else if (target.getX() < robotPosition.get(0)) {
            return ! (obstacles.getX() > robotPosition.get(0) + 9) && ! (obstacles.getX() < target.getX() - 9);
        }
        return true;
    }

    private static boolean tryEquation(Path path, positionInformationRecord obstacles, positionInformationRecord target, ArrayList<Float> robot) {
        float obstacleDistance;
        float slope;

        float p = obstacles.getX();
        float q = obstacles.getY();

        if (! checkDomain(obstacles, target, robot)) {
            return true;
        }

        switch (path) {
            case Direct:


                slope = tangent(path, obstacles, target, robot);

                float intersectX = (p + q * slope) / (slope * slope + 1);

                float intersectY = slope * intersectX;

                obstacleDistance = findDistance(p, q, intersectX, intersectY);

                System.out.println("Distance: " + obstacleDistance);
                return ! (obstacleDistance < 9.5f);

            case Curved:

                float a = deltaX(target, robot);
                float b = deltaY(target, robot);

                slope = tangent(path, obstacles, target, robot);

                if (abs(p - a) < 0.001f) //This value is simulated infinity (i.e. p and a are nearly equal)
                {
                    //Checks if the "infinity" point is at least the robot's width away from the vertex
                    return ! (abs(q - b) < 9.5f);
                }

                if (slope > 0) //Obstacle is out of the domain of the ellipse
                {

                    if (hypot(p - a, q - b) > 12.727f) //Determines if the point is outside the full size of the robot
                    {
                        return true;
                    }
                }

                double asqr = pow(a, 2);
                double bsqr = pow(a, 2);


                intersectX = a - (float) sqrt((asqr * bsqr / (bsqr + (asqr * pow(q, 2) / pow((p - a), 2)))));
                intersectY = q / (p - a) * (intersectX - a);

                obstacleDistance = findDistance(p, q, intersectX, intersectY);

                System.out.println("Distance: " + obstacleDistance);

                return ! (obstacleDistance < 9.5f);

            case XthenY:

                return ! (abs(p - target.getX()) < 9.5f) || ! (abs(q - robot.get(1)) < 9.5f);

            case YthenX:

                return ! (abs(p - robot.get(0)) < 9.5f) || ! (abs(q - target.getY()) < 9.5f);

            default: {
                return false;
            }
        }
    }

    private static Path checkPath(org.firstinspires.ftc.teamcode.odometry.Telemetry telemetry, Path[] paths, List<positionInformationRecord> obstacles, positionInformationRecord target, ArrayList<Float> robot) {
        int j;

        for (Path path : paths) {
            j = 0;
            telemetry.print(""+j);
            while (j < obstacles.size()) {
                telemetry.print(""+j);
                if (! tryEquation(path, obstacles.get(j), target, robot)) {
                    telemetry.print("Obstacle: " + obstacles.get(j));
                    //System.out.println("Path: \"" + path + "\" is not safe\n");
                    j = obstacles.size();
                }
                else if (j == obstacles.size() - 1) {
                    return path;
                }
                j++;
            }
        }
        throw new IllegalStateException("Illegal Path Type");
    }

    public List<positionInformationRecord> getTargetCoords() {
        return targetCoords;
    }

    public ArrayList<Float> getRobotPosition() {
        return robotPosition;
    }

    public void setRobotPosition(final ArrayList<Float> robotPosition) {
        this.robotPosition = robotPosition;
    }

    public int getTargetIndex() {
        return targetIndex;
    }

    public void setTargetIndex(final int targetIndex) {
        this.targetIndex = targetIndex;
    }

    private List<positionInformationRecord> setObstacleCoords() {
        File file = new File(Environment.getExternalStorageDirectory().getPath() + "/Download/ObstacleLocation.txt");

        return setTargetCoords(file);

    }

    private List<positionInformationRecord> setTargetCoords() {
        File file = new File(Environment.getExternalStorageDirectory().getPath() + "/Download/TargetLocation.txt");
        return setTargetCoords(file);

    }

    private List<positionInformationRecord> setTargetCoords(File file) {

        List<positionInformationRecord> foo = new ArrayList<>();
        try (FileReader fileReader = new FileReader(file)) {

            BufferedReader bufferedReader = new BufferedReader(fileReader);
            String line;

            while ((line = bufferedReader.readLine()) != null && ! line.equals("")) {
                String[] split = line.split(",");
                positionInformationRecord data = new positionInformationRecord(Float.valueOf(split[1]), Float.valueOf(split[2])
                        , Float.valueOf(split[3]), Float.valueOf(split[4])
                        , Float.valueOf(split[5]), Float.valueOf(split[6]));

                foo.add(data);


            }

        } catch (IOException e) {
            e.printStackTrace();
        }
        return foo;
    }

    public Path pathFinding(org.firstinspires.ftc.teamcode.odometry.Telemetry telemetry) {
        telemetry.print("IDK whats going on here");
        //Instantiate
        Path determinedPath;

        telemetry.print("Targets: " + targetCoords.get(targetIndex));

        try {

            determinedPath = checkPath(telemetry, Path.values(), obstacleCoords, targetCoords.get(targetIndex), robotPosition);
            return determinedPath;

        } catch (Exception e) {
            e.printStackTrace();
        }
        // FIXME: 11/21/2019
        telemetry.print("all paths fail");
        return null;
    }

    public double[] runtimePathCheck(final Path currentPath)
    {
        double[] distance = getDistanceAway(currentPath);
        return distance;
    }


    //calculates the theta of the relative angle between the location of the robot and the
    private double calculateAngleTurnTowards()
    {
        double xDisplacement = targetCoords.get(targetIndex).getX() - robotPosition.get(0);
        double yDisplacement = targetCoords.get(targetIndex).getY() - robotPosition.get(1);

        double angle;
        try {
            angle = atan(yDisplacement/xDisplacement);
        } catch (Exception e) {
            angle = copySign(90,yDisplacement);
        }
        if(angle == 0 && xDisplacement < 0)
        {
            angle = 180;
        }
        return angle;
    }

    public float checkAngle(float heading)
    {
       double angle = calculateAngleTurnTowards();
        heading %= 180;
        float target = 0;
        target = (float) angle - heading;
        return target;
    }

    @NotNull
    @Contract("_ -> new")
    private double[] getDistanceAway(@NotNull PathFinder.Path currentPath)
    {
        DistanceUnit distanceUnit = DistanceUnit.INCH;

        switch (currentPath)
        {

            case Direct:

                double hyp = hypot(robotPosition.get(0).doubleValue() - targetCoords.get(targetIndex).getX(), robotPosition.get(1) - targetCoords.get(targetIndex).getY());
                return new double[]{distanceUnit.toMm(hyp)};

            case Curved:
                // TODO: 11/25/2019 elliptical motion
                break;

            case XthenY:
                double first = robotPosition.get(0).doubleValue() - targetCoords.get(targetIndex).getX();
                double second = robotPosition.get(1).doubleValue() - targetCoords.get(targetIndex).getY();

                first = distanceUnit.toMm(first);
                second = distanceUnit.toMm(second);

                return new double[]{first , second};

            case YthenX:
                first = robotPosition.get(0).doubleValue() - targetCoords.get(targetIndex).getX();
                second = robotPosition.get(1).doubleValue() - targetCoords.get(targetIndex).getY();

                first = distanceUnit.toMm(first);
                second = distanceUnit.toMm(second);

                return new double[]{second , first};

            default:
                throw new IllegalStateException("Unexpected value: " + currentPath);

        }
        return new double[]{0.0,0.0};
    }

     public enum Path {Direct, Curved, XthenY, YthenX}

    /**
     *
     */
    private static class positionInformationRecord {
        private final float x;
        private final float y;
        private final float z;
        private final float roll;
        private final float pitch;
        private final float heading;

        private positionInformationRecord(final float x, final float y, final float z, final float roll, final float pitch, final float heading) {

            this.x = x;
            this.y = y;
            this.z = z;
            this.roll = roll;
            this.pitch = pitch;
            this.heading = heading;
        }

        public float getX() {
            return x;
        }

        public float getY() {
            return y;
        }

        public float getZ() {
            return z;
        }

        public float getRoll() {
            return roll;
        }

        public float getPitch() {
            return pitch;
        }

        public float getHeading() {
            return heading;
        }

        @NotNull
        @Override
        public String toString() {
            return "FOO{" +
                    "x=" + x +
                    ", y=" + y +
                    ", z=" + z +
                    ", roll=" + roll +
                    ", pitch=" + pitch +
                    ", heading=" + heading +
                    '}';
        }
    }
}


