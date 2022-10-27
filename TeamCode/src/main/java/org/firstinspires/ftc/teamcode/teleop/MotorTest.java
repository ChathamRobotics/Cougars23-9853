package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OurBot;


/*
    TODO





 */

@TeleOp(name="Motor Test")
public class MotorTest extends LinearOpMode {
    private final OurBot robot = new OurBot();
    private double basePower = 0.8;






    /*
    The main loop for the controller

     */
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Let OurRobot do the heavy lifting of getting and initializing the hardware
        robot.init(hardwareMap);
        DcMotor[] motors = new DcMotor[]{robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack};
        int index = 0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();



        // Run until the end of the match (driver presses STOP)
        while(opModeIsActive())
        {


            if(gamepad1.dpad_up)
            {
                index = (index + 1)%4;

            }

            motors[index].setPower(gamepad1.left_stick_y);


            //updates control hub
            telemetry.addData("Motor #", index);
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            telemetry.update();


        }

    }
}
