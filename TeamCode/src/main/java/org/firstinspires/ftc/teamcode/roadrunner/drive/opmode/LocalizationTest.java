package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {

    private TrajectoryFollower follower;
    public DcMotor leftRotation = null;
    public DcMotor rightRotation = null;
    public DcMotor arm = null;
    public Servo claw = null;

    double basePower = 0.5;
    double power;
    boolean backwards = false;
    int rotationRightTarget;
    int rotationLeftTarget;

    int lowPositionAuton = -710;
    int midPositionAuton = -640;
    int highPositionAuton = 0;

    public Servo clawRotation;

    double scale = 0.75;



    //have to check these
    int lowPositionNormal = 50;
    int midPositionNormal = 150;
    int highPositionNormal = 950;

    int lowPosition = lowPositionAuton;
    int highPosition = highPositionAuton;
    @Override
    public void runOpMode() throws InterruptedException {

        //get mec drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //get the claws
        clawRotation = hardwareMap.get(Servo.class, "clawRotation");
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        leftRotation = hardwareMap.get(DcMotor.class, "leftRotation");
        rightRotation = hardwareMap.get(DcMotor.class, "rightRotation");

        //set direction for separate motors
        leftRotation.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRotation.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);


        //set power for extra arms
        arm.setPower(0);
        leftRotation.setPower(0);
        rightRotation.setPower(0);

        //reset encoders
        rightRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set their target position
        leftRotation.setTargetPosition(leftRotation.getCurrentPosition());
        rightRotation.setTargetPosition(rightRotation.getCurrentPosition());

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //set zero power behavior
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //set run to position
        leftRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //set power
        leftRotation.setPower(0.4);
        rightRotation.setPower(0.4);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x * scale
                    )
            );

            if((gamepad2.left_stick_y > 0 && arm.getCurrentPosition() > 5) ||(gamepad2.left_stick_y < 0 && arm.getCurrentPosition() < 2239) ){
                arm.setPower(-(gamepad2.left_stick_y * 0.7));
            }else{
                arm.setPower(0);
            }


            //sets so that you can change macro based on if you did auton before or not
            if(gamepad2.y && gamepad2.b){
                highPosition = highPositionAuton;
                lowPosition = lowPositionAuton;
            }else if (gamepad2.x && gamepad2.a){
                highPosition = highPositionNormal;
                lowPosition = lowPositionNormal;
            }

            //controls power
            if(gamepad1.triangle)
            {
                scale = 1;
            }
            else if(gamepad1.circle)
            {
                scale = 0.75;
            }
            else if(gamepad1.cross)
            {
                scale = 0.5;
            }


            //macros for controller
            if(gamepad2.y)
            {

                //goes up
                rotationLeftTarget = highPosition;
                rotationRightTarget = highPosition;
            }else if(gamepad2.a)
            {

                //goes down
                rotationLeftTarget = lowPosition;
                rotationRightTarget = lowPosition;
            }




            //controls claw
            if(gamepad2.right_trigger > 0)
            {
                //closed position = 0.82
                claw.setPosition(0.7);
            }else if (gamepad2.left_trigger > 0)
            {
                //open position = 0.53556
                if(leftRotation.getCurrentPosition() < -300 )
                {
                    claw.setPosition(0.35);

                }else{
                    claw.setPosition(0.55);
                }

            }


            //manually controls arm rotation position
            if(gamepad2.dpad_up){
                rotationLeftTarget += 5;
                rotationRightTarget += 5;
            }else if(gamepad2.dpad_down){
                rotationLeftTarget -= 5;
                rotationRightTarget -=5;
            }

            if(gamepad2.dpad_left)
            {
                clawRotation.setPosition(clawRotation.getPosition() + 0.01);
            }else if(gamepad2.dpad_right){
                clawRotation.setPosition(clawRotation.getPosition() - 0.01);
            }


            //actively updates target position
            leftRotation.setTargetPosition(rotationLeftTarget);
            rightRotation.setTargetPosition(rotationRightTarget);

            //add speed for moving arm
            if(Math.abs(leftRotation.getCurrentPosition() - leftRotation.getTargetPosition()) >= 5 )
            {

                leftRotation.setPower(0.5);
                rightRotation.setPower(0.5);
            }else{
                leftRotation.setPower(0.6);
                rightRotation.setPower(0.6);
            }

            //if robot going down
            if(leftRotation.getCurrentPosition() < lowPosition + 400 && leftRotation.getTargetPosition() == lowPosition){
                clawRotation.setPosition(0.83);
            }else if (highPosition - leftRotation.getCurrentPosition() < 400 && leftRotation.getTargetPosition() == highPosition){
                clawRotation.setPosition(0.37);
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
