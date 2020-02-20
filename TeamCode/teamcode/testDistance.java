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

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class testDistance implements DistanceSensor {
    public void setDistance(final double distance)
    {
        this.distance = distance;
    }

    private double distance = 200;

    /**
     * Returns the current distance in the indicated distance units
     *
     * @param unit the unit of distance in which the result should be returned
     *
     * @return the current distance sas measured by the sensor. If no reading is available
     * (perhaps the sensor is out of range), then {@link #distanceOutOfRange} is
     * returned;
     */
    @Override
    public double getDistance(final DistanceUnit unit)
    {
        return 200;
    }


    /**
     * Returns an indication of the manufacturer of this device.
     *
     * @return the device's manufacturer
     */
    @Override
    public Manufacturer getManufacturer()
    {
        return null;
    }

    /**
     * Returns a string suitable for display to the user as to the type of device.
     * Note that this is a device-type-specific name; it has nothing to do with the
     * name by which a user might have configured the device in a robot configuration.
     *
     * @return device manufacturer and name
     */
    @Override
    public String getDeviceName()
    {
        return null;
    }

    /**
     * Get connection information about this device in a human readable format
     *
     * @return connection info
     */
    @Override
    public String getConnectionInfo()
    {
        return null;
    }

    /**
     * Version
     *
     * @return get the version of this device
     */
    @Override
    public int getVersion()
    {
        return 0;
    }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.
     * For example, motors will reset the their direction to 'forward'.
     */
    @Override
    public void resetDeviceConfigurationForOpMode()
    {

    }

    /**
     * Closes this device
     */
    @Override
    public void close()
    {

    }
}
