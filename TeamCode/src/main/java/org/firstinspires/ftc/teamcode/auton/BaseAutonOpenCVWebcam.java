package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OurBot;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp
@Disabled
public class BaseAutonOpenCVWebcam extends LinearOpMode
{
    OpenCvWebcam webcam;
    SamplePipeline pipeline;

    //init the robot and its motors/servos
    OurBot robot = new OurBot();

    //gets our run time in case we want to use time
    public final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });



    }

    //moves arm via encoders
    protected void encoderLift(double power, double inchesTarget, double timeout)
    {
        int target;


        if(opModeIsActive())
        {

            target = (int)(inchesTarget * OurBot.COUNTS_PER_INCH_ARM);

            robot.arm.setTargetPosition(target);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            robot.arm.setPower(power);

            while (opModeIsActive() && (runtime.seconds() < timeout) && (robot.arm.isBusy()))
            {



                // Display it for the driver.
                telemetry.addData("Running to", "%7d", target);
                telemetry.addData("Currently at", "%7d", robot.arm.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion once the loop ends (it reached target position)
            robot.arm.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
    }


    //uses speed and the amount we want to move each side by to move robot
    protected void encoderDrive(double power, double leftFrontInches, double leftBackInches, double rightFrontInches, double rightBackInches, double timeout, boolean scaled)
    {

        //set up variables
        int leftFrontStart;
        int leftBackStart;
        int rightFrontStart;
        int rightBackStart;

        int leftFrontTarget;
        int leftBackTarget;
        int rightFrontTarget;
        int rightBackTarget;

        double leftFrontScale;
        double leftBackScale;
        double rightFrontScale;
        double rightBackScale;


        if(opModeIsActive())
        {
            //set starting variables
            leftFrontStart = robot.leftFront.getCurrentPosition();
            leftBackStart = robot.leftBack.getCurrentPosition();
            rightFrontStart = robot.rightFront.getCurrentPosition();
            rightBackStart = robot.rightBack.getCurrentPosition();



            // Determine new target position, and pass to motor controller
            leftFrontTarget = leftFrontStart + (int) (leftFrontInches * OurBot.COUNTS_PER_INCH);
            leftBackTarget = leftBackStart + (int) (leftBackInches * OurBot.COUNTS_PER_INCH);
            rightFrontTarget = rightFrontStart + (int) (rightFrontInches * OurBot.COUNTS_PER_INCH);
            rightBackTarget = rightBackStart + (int) (rightBackInches * OurBot.COUNTS_PER_INCH);

            //set the robot's new position that it has to get to
            robot.leftFront.setTargetPosition(leftFrontTarget);
            robot.leftBack.setTargetPosition(leftBackTarget);
            robot.rightFront.setTargetPosition(rightFrontTarget);
            robot.rightBack.setTargetPosition(rightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            power = Math.abs(power);
            robot.leftFront.setPower(power);
            robot.leftBack.setPower(power);
            robot.rightFront.setPower(power);
            robot.rightBack.setPower(power);


            /*

            will keep updating telemetry data until:
            1. opmode is ended by driver
            2. move runs out of time originally set
            3. the robot is in its target position

            */
            while (opModeIsActive() && (runtime.seconds() < timeout) && (robot.leftFront.isBusy()
                    || robot.leftBack.isBusy() || robot.rightFront.isBusy() || robot.rightBack.isBusy()))
            {
                if(scaled)
                {
                    leftFrontScale = Math.max(1 - ((double)robot.leftFront.getCurrentPosition() - leftFrontStart) / (leftFrontTarget - leftFrontStart), 0.25);
                    leftBackScale = Math.max(1 - ((double)robot.leftBack.getCurrentPosition() - leftBackStart) / (leftBackTarget - leftBackStart), 0.25);
                    rightFrontScale = Math.max(1 - ((double)robot.rightFront.getCurrentPosition() - rightFrontStart) / (rightFrontTarget - rightFrontStart), 0.25);
                    rightBackScale = Math.max(1 - ((double)robot.rightBack.getCurrentPosition() - rightBackStart) / (rightBackTarget - rightBackStart), 0.25);

                }else{
                    leftFrontScale = 1;
                    leftBackScale = 1;
                    rightFrontScale = 1;
                    rightBackScale = 1;



                }
                robot.leftFront.setPower(power * leftFrontScale);
                robot.leftBack.setPower(power * leftBackScale);
                robot.rightFront.setPower(power * rightFrontScale);
                robot.rightBack.setPower(power * rightBackScale);



                // Display it for the driver.
                telemetry.addData("Running to", "%7d, %7d, %7d, %7d", leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget);
                telemetry.addData("Currently at", "%7d, %7d, %7d, %7d", robot.leftFront.getCurrentPosition(), robot.leftBack.getCurrentPosition(), robot.rightFront.getCurrentPosition(), robot.rightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion once the loop ends (it reached target position)
            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }


    }

    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */



        Point topLeft = new Point(40, 180);
        Point bottomRight = new Point(60, 205);


        Mat region;

        private volatile int redAverage;
        private volatile int greenAverage;
        private volatile int blueAverage;







        @Override
        public Mat processFrame(Mat input)
        {

            //get the region we want
            region = input.submat(new Rect(topLeft, bottomRight) );


            //compute averages for the channels to test
            redAverage = (int) Core.mean(region).val[0];
            greenAverage = (int) Core.mean(region).val[1];
            blueAverage = (int) Core.mean(region).val[2];

            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the region
             */
            Imgproc.rectangle(
                    input,
                    topLeft,
                    bottomRight,
                    new Scalar(0, 255, 0), 4);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
        public int getRedAverage()
        {
            return redAverage;
        }

        public int getGreenAverage()
        {
            return greenAverage;
        }

        public int getBlueAverage()
        {
            return blueAverage;
        }
    }
}