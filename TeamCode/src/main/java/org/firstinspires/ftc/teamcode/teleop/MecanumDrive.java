package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OurBot;


@TeleOp
public class MecanumDrive extends LinearOpMode {

    //set up variables
    OurBot robot = new OurBot();
    double basePower = 0.5;
    double power;
    boolean backwards = false;
    int rotationRightTarget;
    int rotationLeftTarget;

    public Servo clawRotation;

    int lowPositionAuton = -710;
    int highPositionAuton = 0;


    //have to check these
    int lowPositionNormal = 50;
    int highPositionNormal = 950;

    int lowPosition = lowPositionAuton;
    int highPosition = highPositionAuton;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        clawRotation = hardwareMap.get(Servo.class, "clawRotation");



        robot.leftRotation.setPower(0);


        robot.rightRotation.setPower(0);

        waitForStart();
        power = basePower;
        clawRotation.setPosition(0);
        while(opModeIsActive())
        {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;


            //macros for changing direction
            if(gamepad1.dpad_up){
                backwards = false;
            }else if(gamepad1.dpad_down){
                backwards = true;
            }


            //controls if the controls are backwards
            if(backwards){

                robot.leftFront.setPower(-(power*(y + x - rx)));
                robot.leftBack.setPower(-(power*(y - x - rx)));
                robot.rightFront.setPower(-(power*(y - x + rx)));
                robot.rightBack.setPower(-(power*(y + x + rx)));

            }else{
                robot.leftFront.setPower(power*(y + x + rx));
                robot.leftBack.setPower(power*(y - x + rx));
                robot.rightFront.setPower(power*(y - x - rx));
                robot.rightBack.setPower(power*(y + x - rx));

            }




            //arm max = 2239
            //controls how far the arm goes
            if((gamepad2.left_stick_y > 0 && robot.arm.getCurrentPosition() > 5) ||(gamepad2.left_stick_y < 0 && robot.arm.getCurrentPosition() < 2239) ){
                robot.arm.setPower(-(gamepad2.left_stick_y * 0.7));
            }else{
                robot.arm.setPower(0);
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
                power = basePower + 0.3;
            }
            else if(gamepad1.circle)
            {
                power = basePower;
            }
            else if(gamepad1.cross)
            {
                power = basePower - 0.3;
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
                robot.claw.setPosition(0.7);
            }else if (gamepad2.left_trigger > 0)
            {
                //open position = 0.53556
                if(robot.leftRotation.getCurrentPosition() < -300 )
                {
                    robot.claw.setPosition(0.35);

                }else{
                    robot.claw.setPosition(0.55);
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
            robot.leftRotation.setTargetPosition(rotationLeftTarget);
            robot.rightRotation.setTargetPosition(rotationRightTarget);

            //add speed for moving arm
            if(Math.abs(robot.leftRotation.getCurrentPosition() - robot.leftRotation.getTargetPosition()) >= 5 )
            {

                robot.leftRotation.setPower(0.5);
                robot.rightRotation.setPower(0.5);
            }else{
                robot.leftRotation.setPower(0.6);
                robot.rightRotation.setPower(0.6);
            }

            //if robot going down
            if(robot.leftRotation.getCurrentPosition() < lowPosition + 400 && robot.leftRotation.getTargetPosition() == lowPosition){
                clawRotation.setPosition(0.83);
            }else if (highPosition - robot.leftRotation.getCurrentPosition() < 400 && robot.leftRotation.getTargetPosition() == highPosition){
                clawRotation.setPosition(0.37);
            }


            telemetry.addData("Arm position", robot.arm.getCurrentPosition());
            telemetry.addData("Backwards", backwards);
            telemetry.addData("High rotational position", highPosition);
            telemetry.addData("claw rotation position", clawRotation.getPosition());
            telemetry.addData("motor1 position", robot.leftFront.getCurrentPosition());
            telemetry.addData("motor2 position", robot.leftBack.getCurrentPosition());
            telemetry.update();


        }

    }
}
