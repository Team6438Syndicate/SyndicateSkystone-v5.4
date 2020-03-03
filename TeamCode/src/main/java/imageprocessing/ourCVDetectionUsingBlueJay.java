package imageprocessing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotMovements;

import java.util.Arrays;

import detectors.FoundationPipeline.Pipeline;
import detectors.FoundationPipeline.SkyStone;
import detectors.OpenCvDetector;

@TeleOp(name = "OpenCv avec le geai bleu", group="BlueJayAdditions")
public class ourCVDetectionUsingBlueJay extends RobotMovements
{
    private OpenCvDetector fieldElementDetector;

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException
    {
        try
        {
            initRobot(hardwareMap,true);
            fieldElementDetector = new OpenCvDetector(this,true,hardwareMap);
            fieldElementDetector.start();

            waitForStart();

		/*
		To toggle individual element detection, use the following.
		Note that the detector must still be enabled for any element detection to occur
		 */
            Pipeline.doSkyStones = true;
            Pipeline.doStones = false;
            Pipeline.doFoundations = false;

            SkyStone[] skyStones;
            //Stone[] stones = fieldElementDetector.getStones();
            //Foundation[] foundations = fieldElementDetector.getFoundations();

            while (opModeIsActive())
            {
                skyStones = fieldElementDetector.getSkyStones();

                for (SkyStone detection : skyStones)
                {
                    if (detection.y < 111)
                    {
                        telemetry.addData("Skystone Data: ", "X=" + detection.x + ", Y= " + detection.y);

                    }

                    if(detection.x < fieldElementDetector.getWidth()/3.0)
                    {
                        telemetry.speak("Left");
                    }
                    else if(detection.x > 2 * fieldElementDetector.getWidth()/3.0)
                    {
                        telemetry.speak("Right");

                    }
                    else
                    {
                        telemetry.speak("Center");

                    }



                }

                telemetry.update();

                //telemetry.speak(Arrays.toString(skyStones));
            }

            fieldElementDetector.stop();

        } catch (Exception e)
        {
            telemetry.addData("Error:" , Arrays.toString(e.getStackTrace()));
            telemetry.update();
            sleep(100000);
        }

    }
}
