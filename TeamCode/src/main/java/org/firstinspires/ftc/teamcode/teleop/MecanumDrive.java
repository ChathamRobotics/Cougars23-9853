package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    public DcMotor rotation1;
    public DcMotor rotation2;
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


        robot.leftRotation.setPower(0.3);


        robot.rightRotation.setPower(0.3);

        waitForStart();
        power = basePower;
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
                if(rotation1.getCurrentPosition() < -300 )
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


            //actively updates target position
            rotation1.setTargetPosition(rotationLeftTarget);
            rotation2.setTargetPosition(rotationRightTarget);

            telemetry.addData("Arm position", robot.arm.getCurrentPosition());
            telemetry.addData("Backwards", backwards);
            telemetry.addData("High rotational position", highPosition);
            telemetry.update();


        }

    }
}
