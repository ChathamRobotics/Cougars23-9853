package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OurBot;


@TeleOp(name="Main Drive")
public class MainDrive extends LinearOpMode{
    private final OurBot robot = new OurBot();
    private final double basePower = 0.3;
@Override

     public void runOpMode() {

        //set initial variables
        double power = basePower;
        double lowPower = basePower - 0.2;
        double highPower = basePower + 0.2;
    String orientation;
    boolean backwards = false;

        //add initial message
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Let OurRobot do the heavy lifting of getting and initializing the hardware
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while(opModeIsActive()) {

            //sets power
            if (gamepad1.dpad_down) {
                if (power == basePower) {
                    power = lowPower;
                } else {
                    power = basePower;
                }
            }
            else if (gamepad1.dpad_up) {
                if (power == basePower) {
                    power = highPower;
                } else {
                    power = basePower;
                }
            }


            //changes driver POV - if robot is backwards, sets controls to account for that
            if(gamepad1.dpad_right) {
                backwards = false;
            }else if(gamepad1.dpad_left)
            {
                backwards = true;
            }

            //gets input from controllers to configure which power to use
            double drive = -(gamepad1.left_stick_y);
            double driveX = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            //sets power for normal drive
            double leftPower = Range.clip(drive + turn, -1, 1) * power;
            double rightPower = Range.clip(drive - turn, -1, 1) * power;

            //gets values for triggers
            double rightTrigger = gamepad1.right_trigger;
            double leftTrigger = gamepad1.left_trigger;

            // Strafe right
            if (rightTrigger > 0) {



                //if robot is backwards
                if(backwards)
                {
                    robot.rightBack.setPower(-(power * rightTrigger));
                    robot.rightFront.setPower(power * rightTrigger);
                    robot.leftBack.setPower(power * rightTrigger);
                    robot.leftFront.setPower(-(power * rightTrigger));

                }else{
                    //if robot is normal orientation
                    // Left wheels going out
                    // Right wheels going in
                    robot.leftFront.setPower(power * rightTrigger);
                    robot.leftBack.setPower(-(power * rightTrigger));
                    robot.rightFront.setPower(-(power * rightTrigger));
                    robot.rightBack.setPower(power * rightTrigger);

                }



            }
            // Strafe left
            else if (leftTrigger > 0) {



                //if robot is backwards
                if(backwards)
                {
                    robot.rightBack.setPower(power * leftTrigger);
                    robot.rightFront.setPower(-(power * leftTrigger));
                    robot.leftBack.setPower(-(power * leftTrigger));
                    robot.leftFront.setPower(power * leftTrigger);
                }else{
                    //if robot is straight
                    // Left wheels going in
                    // Right wheels going out
                    robot.leftFront.setPower(-(power * leftTrigger));
                    robot.leftBack.setPower(power * leftTrigger);
                    robot.rightFront.setPower(power * leftTrigger);
                    robot.rightBack.setPower(-(power * leftTrigger));

                }

            } else {

                //if robot is backwards
                if (backwards) {
                    robot.leftFront.setPower(-rightPower);
                    robot.leftBack.setPower(-rightPower);
                    robot.rightFront.setPower(-leftPower);
                    robot.rightBack.setPower(-leftPower);
                } else {
                        // Sets power for main drive
                        //if robot is forward
                        robot.leftFront.setPower(leftPower);
                        robot.leftBack.setPower(leftPower);
                        robot.rightFront.setPower(rightPower);
                        robot.rightBack.setPower(rightPower);
                    }


            }


            // Updates telemetry
            if(backwards)
            {
                orientation = "backwards";
            }else{
                orientation = "forwards";
            }
            telemetry.addData("Main Power", power);
            telemetry.addData("Orientation", orientation  );
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Right Stick x", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}