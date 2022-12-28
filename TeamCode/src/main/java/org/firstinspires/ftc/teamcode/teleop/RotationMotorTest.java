package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp
public class RotationMotorTest extends LinearOpMode {
    public DcMotor rotation;
    public DcMotor arm;
    public boolean dual = true;

    //1 = left motor ---- 2 = right motor
    int rotationTarget;
    int armTarget;

    @Override
    public void runOpMode() throws InterruptedException {
        rotation = hardwareMap.get(DcMotor.class, "rotation");
        rotation.setDirection(DcMotorSimple.Direction.FORWARD);
        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationTarget = rotation.getCurrentPosition();
        rotation.setTargetPosition(rotationTarget);
        rotation.setPower(0.3);


        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armTarget = arm.getTargetPosition();
        arm.setTargetPosition(armTarget);
        arm.setPower(0.3);

        telemetry.addData("Status", "Initialized");
        telemetry.update();




        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.y)
            {
                dual = true;
            }else if(gamepad1.a)
            {
                dual = false;
            }



            /*if(dual)
            {
                leftRotation.setPower(-gamepad1.left_stick_y);
                rightRotation.setPower(-gamepad1.left_stick_y);

            }else{
                if(currentMotor == 1){
                    leftRotation.setPower(-gamepad1.left_stick_y);

                }else{
                    rightRotation.setPower(-gamepad1.left_stick_y);
                }
            }*/

            rotationTarget += -gamepad1.left_stick_y * 25;
            armTarget += -gamepad1.right_stick_y * 25;
            arm.setTargetPosition(armTarget);
            rotation.setTargetPosition(rotationTarget);

            telemetry.addData("Rotation Motor Position", rotation.getCurrentPosition() );
            telemetry.addData("Rotation Motor Target", rotationTarget);

            telemetry.addData("Arm Motor Position", arm.getCurrentPosition());
            telemetry.addData("Arm Motor Target", armTarget);
            telemetry.update();


        }








    }
}
