package org.firstinspires.ftc.teamcode.prototypes;

import android.app.Application;
import android.content.Context;
import android.os.Environment;
import android.widget.Toast;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.OpenCV.DogeCV.SkystoneDetectorExample;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.lang.reflect.Method;
import java.util.Locale;

import static java.lang.Thread.sleep;

public class CVtest
{
    static OpenCvCamera webcam;
    private static SkystoneDetector skyStoneDetector;
    private static boolean test = false;
    private static int counter;

    private enum locations {Left, Center, Right}

    static CVtest.locations position;
    private static HardwareMap hardwareMap = new HardwareMap(getContext());



    public static Context getContext() {
        try {
            final Class<?> activityThreadClass =
                    Class.forName("android.app.ActivityThread");
            //find and load the main activity method
            final Method method = activityThreadClass.getMethod("currentApplication");
            return (Application) method.invoke(null, (Object[]) null);
        } catch (final java.lang.Throwable e) {
            // handle exception
            throw new IllegalArgumentException("No context could be retrieved!");
        }
    }
    public static void runOpMode(Context context) throws InterruptedException
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = context.getResources().getIdentifier("cameraMonitorViewId", "id", context.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK,cameraMonitorViewId);// OR...  Do Not Activate the Camera Monitor View
        //webcam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        webcam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        skyStoneDetector = new SkystoneDetector();
        webcam.setPipeline(skyStoneDetector);

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);

        /*
         * Wait for the user to press start on the Driver Station
         */

        while (counter < 1000)
        {

            if (skyStoneDetector.getScreenPosition().x < 150)
            {
                position = CVtest.locations.Left;
                Toast.makeText(context,"Left",Toast.LENGTH_SHORT).show();
                sleep(500);
            }
            else if (skyStoneDetector.getScreenPosition().x > 150 && skyStoneDetector.getScreenPosition().x < 200)
            {
                position = CVtest.locations.Center;
                Toast.makeText(context,"Center",Toast.LENGTH_SHORT).show();

                sleep(500);
            }
            else
            {
                position = CVtest.locations.Right;
                Toast.makeText(context,"Right",Toast.LENGTH_SHORT).show();

                sleep(500);
            }
            counter++;

        }
        //Close the cam when the op mode stops
        webcam.closeCameraDevice();

    }

}
