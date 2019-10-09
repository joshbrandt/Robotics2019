package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Encoder Test")
public class autoEncoderTest extends LinearOpMode {

    DcMotor frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;

    boolean flWorking, frWorking, brWorking, blWorking;

    public void runOpMode() {

        frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        backRightMotor = hardwareMap.get(DcMotor.class, "br");
        backLeftMotor = hardwareMap.get(DcMotor.class, "bl");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        runIntro();

        flWorking = testMotor(frontLeftMotor, flWorking);
        frWorking = testMotor(frontRightMotor, frWorking);
        blWorking = testMotor(backLeftMotor, blWorking);
        brWorking = testMotor(backRightMotor, brWorking);

        telemetry.clearAll();

        telemetry.addData("Front Right Working?", frWorking);
        telemetry.addData("Front Left Working?", flWorking);
        telemetry.addData("Back Right Working?", brWorking);
        telemetry.addData("Back Left Working?", blWorking);

        telemetry.update();


    }


    public void runIntro() {

        telemetry.addData("Instructions", "Flip the robot over...");
        telemetry.update();

        try {
            sleep(1000);
        } catch (Exception e) {}

        telemetry.addData("Instructions", "Test Initiating in 3");
        telemetry.update();

        try {
            sleep(1000);
        } catch (Exception e) {}

        telemetry.clearAll();

        telemetry.addData("Instructions", "2");
        telemetry.update();

        try {
            sleep(1000);
        } catch (Exception e) {}

        telemetry.clearAll();

        telemetry.addData("Instructions", "1");
        telemetry.update();

        try {
            sleep(1000);
        } catch (Exception e) {}

        telemetry.clearAll();

        telemetry.addData("Notice", "Test Running...");
        telemetry.update();

    }

    public boolean testMotor(DcMotor motor, boolean motorWorking) {

        int[] pos = new int[2];

        motor.setPower(1);

        try {
            sleep(1000);
        } catch (Exception e) {}

        motor.setPower(0);

        pos[0] = motor.getCurrentPosition();

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setPower(-1);

        try {
            sleep(1000);
        } catch (Exception e) {}

        motor.setPower(0);

        pos[1] = motor.getCurrentPosition();

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       if (pos[0] > 200 && pos[1] < -200) motorWorking = true;

       return motorWorking;

    }

}
