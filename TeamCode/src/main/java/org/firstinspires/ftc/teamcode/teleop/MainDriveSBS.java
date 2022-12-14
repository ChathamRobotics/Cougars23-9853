package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OurBot;


@TeleOp(name = "Main Drive SBS")
public class MainDriveSBS extends LinearOpMode {
    private final OurBot robot = new OurBot();
    private final double basePower = 0.1;

    @Override

    public void runOpMode() {

        //set initial variables
        double drivePower = basePower;
        double armPower = basePower;
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

        while (opModeIsActive()) {

            /* ----------------------
               | Gamepad 1 Controls |
               ---------------------- */


            /*
            GAMEPAD 1 CONTROLS
             */
            //sets power


            //changes driver POV - if robot is backwards, sets controls to account for that
            if (gamepad1.dpad_right) {
                backwards = false;
            } else if (gamepad1.dpad_left) {
                backwards = true;
            }

            //gets input from controllers to configure which power to use
            double drive = -(gamepad1.left_stick_y);
            double turn = gamepad1.right_stick_x;

            //sets power for normal drive
            double leftPower = Range.clip(drive + turn, -1, 1) * drivePower;
            double rightPower = Range.clip(drive - turn, -1, 1) * drivePower;

            //gets values for triggers
            double rightTrigger = gamepad1.right_trigger;
            double leftTrigger = gamepad1.left_trigger;

            // Strafe right
            if (rightTrigger > 0) {

                //if robot is backwards
                if (backwards) {
                    robot.rightBack.setPower(-(drivePower * rightTrigger));
                    robot.rightFront.setPower(drivePower * rightTrigger);
                    robot.leftBack.setPower(drivePower * rightTrigger);
                    robot.leftFront.setPower(-(drivePower * rightTrigger));

                } else {
                    //if robot is normal orientation
                    // Left wheels going out
                    // Right wheels going in
                    robot.leftFront.setPower(drivePower * rightTrigger);
                    robot.leftBack.setPower(-(drivePower * rightTrigger));
                    robot.rightFront.setPower(-(drivePower * rightTrigger));
                    robot.rightBack.setPower(drivePower * rightTrigger);
                }


            }
            // Strafe left
            else if (leftTrigger > 0) {

                //if robot is backwards
                if (backwards) {
                    robot.rightBack.setPower(drivePower * leftTrigger);
                    robot.rightFront.setPower(-(drivePower * leftTrigger));
                    robot.leftBack.setPower(-(drivePower * leftTrigger));
                    robot.leftFront.setPower(drivePower * leftTrigger);
                } else {
                    //if robot is straight
                    // Left wheels going in
                    // Right wheels going out
                    robot.leftFront.setPower(-(drivePower * leftTrigger));
                    robot.leftBack.setPower(drivePower * leftTrigger);
                    robot.rightFront.setPower(drivePower * leftTrigger);
                    robot.rightBack.setPower(-(drivePower * leftTrigger));

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


            /*
            GAMEPAD 2 CONTROLS
             */


            //controls arm

           /* if(gamepad2.left_stick_y > 0)
            {
                robot.arm.setPower(-gamepad2.left_stick_y * (basePower - 0.2));

            }else if(gamepad2.left_stick_y < 0)
            {
                robot.arm.setPower(-gamepad2.left_stick_y * basePower);
            }*/

            robot.arm.setPower(-gamepad2.left_stick_y * basePower);

            //controls claw
            if (gamepad2.right_trigger > 0) {
                //closed position = 0.82
                robot.claw.setPosition(0.82);
            } else if (gamepad2.left_trigger > 0) {
                //open position = 0.53556
                robot.claw.setPosition(0.53556);
            }


            // Updates telemetry
            if (backwards) {
                orientation = "backwards";
            } else {
                orientation = "forwards";
            }


            telemetry.addData("Main Drive Power", drivePower);
            telemetry.addData("Orientation", orientation);
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Claw Servo position", robot.claw.getPosition());
            telemetry.update();
        }
    }
}