package imageprocessing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import detectors.FoundationPipeline.Foundation;
import detectors.FoundationPipeline.Pipeline;
import detectors.FoundationPipeline.SkyStone;
import detectors.FoundationPipeline.Stone;
import detectors.OpenCvDetector;

/*

    If you're using this library, THANKS! I spent a lot of time on it.

    However, stuff isn't as well-documented as I like...still working on that

    So if you have questions, email me at xchenbox@gmail.com and I will get back to you in about a day (usually)

    Enjoy!

    Below is the code to display to the RC; Thanks, EasyOpenCV!
	If it crashes after about a minute, it's probably because OpenCV is using too much native memory.
	That should not happen much because the MatAllocator recycles all Mats (but not MatofInt)
 */

@TeleOp(name = "CV test", group = "Auto")
public class OpenCVDetection extends OpMode
{


	/*
	You cannot instantiate the actual object here. Must do it either in init() or OnOpmodeStart()
	 However, it's good to make this reference a field
	 */
	OpenCvDetector fieldElementDetector;

	@Override
	public void init() {
		/*
		Makes an OpenCV detector
		 */
		fieldElementDetector = new OpenCvDetector(this);

		/*
		alternatively, use fieldElementDetector = new OpenCvDetector(this,false);
		to run without video and with slightly faster processing
		 */
	}

	/*
	 * Code to run REPEATEDLY when the driver hits INIT
	 */
	@Override
	public void init_loop() {
	}

	/*
	 * Code to run ONCE when the driver hits PLAY
	 */
	@Override
	public void start() {

		/*
		Using start() and stop() will turn the detector on and off. Just like a stove, turn
		it off when not in use! It will consume memory and processing resources.
		 */

		fieldElementDetector.start();

		/*
		To toggle individual element detection, use the following.
		Note that the detector must still be enabled for any element detection to occur
		 */
		Pipeline.doSkyStones = true;
		Pipeline.doStones = false;
		Pipeline.doFoundations = false;

	}

	/*
	 * Code to run REPEATEDLY when the driver hits PLAY
	 */
	@Override
	public void loop() {
		/*
		To request data, use these requests
	    */
		SkyStone[] skyStones = fieldElementDetector.getSkyStones();
		Stone[] stones = fieldElementDetector.getStones();
		Foundation[] foundations = fieldElementDetector.getFoundations();

		/*
		If the detector is not enabled or the specific element is set to false, these
		method calls will return an empty array
		 */

		/*
		Each element object has the center x and y. Just do Skystone.x etc.

		Hold your phone horizontally, with cameras on right. origin is (0,0), on the top left.
		Width is 640, height is 480 pixels. For example, center is (320,240)
		 */
	}

	/*
	 * Code to run ONCE after the driver hits STOP
	 */
	@Override
	public void stop() {
		/*
		To save resources
		 */

		fieldElementDetector.stop();
	}
}
