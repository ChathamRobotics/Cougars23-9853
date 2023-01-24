package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous
@Disabled
public class NewAutonRight extends BaseAuton{
    int rotation2Target;
    int rotation1Target;
    public DcMotor rotation1;
    public DcMotor rotation2;
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() {

        //sets up auton by running parent's method
        super.runOpMode();

        //sets up rotation motor1
        rotation1 = hardwareMap.get(DcMotor.class, "leftRotation");
        rotation1.setDirection(DcMotorSimple.Direction.REVERSE);
        rotation1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotation1Target = rotation1.getCurrentPosition();
        rotation1.setTargetPosition(rotation1Target);
        rotation1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotation1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rotation1.setPower(0.3);

        //sets up rotation motor2
        rotation2 = hardwareMap.get(DcMotor.class, "rightRotation");
        rotation2.setDirection(DcMotorSimple.Direction.REVERSE);
        rotation2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotation2Target  = rotation2.getCurrentPosition();
        rotation2.setTargetPosition(rotation2Target);
        rotation2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotation2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rotation2.setPower(0.3);

        robot.claw.setPosition(0.7);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        //wait for driver to press play
        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        robot.leftFront.setTargetPosition((int) -(robot.COUNTS_PER_INCH*47));
        robot.leftBack.setTargetPosition((int) -(robot.COUNTS_PER_INCH*47));
        robot.rightFront.setTargetPosition((int) -(robot.COUNTS_PER_INCH*47));
        robot.rightBack.setTargetPosition((int) -(robot.COUNTS_PER_INCH*47));
        rotation1.setTargetPosition(500);
        rotation2.setTargetPosition(500);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rotation1.setPower(0.4);
        rotation2.setPower(0.4);
        robot.leftFront.setPower(0.3);
        robot.leftBack.setPower(0.3);
        robot.rightFront.setPower(0.3);
        robot.rightBack.setPower(0.3);

        while(robot.rightBack.isBusy() || robot.leftBack.isBusy() || robot.rightFront.isBusy() || robot.leftFront.isBusy()){
            //
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading", angles.firstAngle);
            telemetry.addData("roll", angles.secondAngle);
            telemetry.addData("pitch", angles.thirdAngle);
            telemetry.update();
        }

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        telemetry.addData("roll", angles.secondAngle);
        telemetry.addData("pitch", angles.thirdAngle);
        telemetry.update();

        sleep(6000);



        rotation1.setTargetPosition(980);
        rotation2.setTargetPosition(980);
        while(rotation1.isBusy() && rotation2.isBusy())
        {
         //
        }
        sleep(500);
        //strafe "left"
        robot.leftFront.setPower(0.1);
        robot.leftBack.setPower(0.1);
        robot.rightFront.setPower(0.1);
        robot.rightBack.setPower(0.1);
        robot.leftFront.setTargetPosition(  robot.leftFront.getCurrentPosition() + (int)(robot.COUNTS_PER_INCH*16));
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition()-(int)(robot.COUNTS_PER_INCH*16));
        robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition()-(int)(robot.COUNTS_PER_INCH*16));
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition()+(int)(robot.COUNTS_PER_INCH*16));




        while(robot.rightBack.isBusy() || robot.leftBack.isBusy() || robot.rightFront.isBusy() || robot.leftFront.isBusy()){
            //
        }



        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

        //get arm ready to extend
        robot.arm.setTargetPosition(2150);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(0.5);

        while(robot.arm.isBusy()){
            //
        }

        //move robot over to prepare score
        rotation1.setPower(0.5);
        rotation2.setPower(0.5);
        robot.leftFront.setPower(0.2);
        robot.leftBack.setPower(0.2);
        robot.rightFront.setPower(0.2);
        robot.rightBack.setPower(0.2);
        robot.leftFront.setTargetPosition(  robot.leftFront.getCurrentPosition() + (int)(robot.COUNTS_PER_INCH*2.5));
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition()+(int)(robot.COUNTS_PER_INCH*2.5));
        robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition()+(int)(robot.COUNTS_PER_INCH*2.5));
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition()+(int)(robot.COUNTS_PER_INCH*2.5));


        while(robot.rightBack.isBusy() || robot.leftBack.isBusy() || robot.rightFront.isBusy() || robot.leftFront.isBusy()){
            //
        }

        //put cone on claw
        sleep(2000);
        robot.claw.setPosition(0.55);
        sleep(2000);
        robot.claw.setPosition(0.55);

        //go down from pole, back up a bit
        robot.leftFront.setTargetPosition(  robot.leftFront.getCurrentPosition() + (int)(robot.COUNTS_PER_INCH*2.5));
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition()+(int)(robot.COUNTS_PER_INCH*2.5));
        robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition()+(int)(robot.COUNTS_PER_INCH*2.5));
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition()+(int)(robot.COUNTS_PER_INCH*2.5));
        robot.arm.setTargetPosition(0);

        while(robot.rightBack.isBusy() || robot.leftBack.isBusy() || robot.rightFront.isBusy() || robot.leftFront.isBusy() || robot.arm.isBusy()){
            telemetry.addData("arm", "bruh we processing");
            telemetry.update();
        }

        /*robot.leftFront.setTargetPosition(  robot.leftFront.getCurrentPosition() - (int)(robot.COUNTS_PER_INCH*.52));
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition()-(int)(robot.COUNTS_PER_INCH*2.5));
        robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition()+(int)(robot.COUNTS_PER_INCH*2.5));
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition()+(int)(robot.COUNTS_PER_INCH*2.5));*/





        //encoderDrive(0.3, 40, 40, 40, 40, 5);



        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.




    }

}
