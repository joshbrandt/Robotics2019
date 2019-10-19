package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Encoder Test")
public class autoEncoderTest extends LinearOpMode {

    boolean flWorking, frWorking, brWorking, blWorking;

    Hardware robot = new Hardware();

    public void runOpMode() {

      robot.init(hardwareMap);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        runIntro();

        flWorking = testMotor(robot.frontLeftMotor, flWorking);

        try {
            sleep(1000);
        } catch (Exception e) {}

        telemetry.clearAll();

        frWorking = testMotor(robot.frontRightMotor, frWorking);

        try {
            sleep(1000);
        } catch (Exception e) {}

        telemetry.clearAll();


        blWorking = testMotor(robot.backLeftMotor, blWorking);

        try {
            sleep(1000);
        } catch (Exception e) {}

        telemetry.clearAll();

        brWorking = testMotor(robot.backRightMotor, brWorking);


        telemetry.addData("Front Right Working?", frWorking);
        telemetry.addData("Front Left Working?", flWorking);
        telemetry.addData("Back Right Working?", brWorking);
        telemetry.addData("Back Left Working?", blWorking);

        telemetry.update();

        try {
            sleep(5000);
        } catch (Exception e) {}


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
            sleep(5000);
        } catch (Exception e) {
        }

        motor.setPower(0);

        pos[0] = motor.getCurrentPosition();

        telemetry.addData("Value Forward", pos[0]);
        telemetry.update();

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setPower(-1);

        try {
            sleep(5000);
        } catch (Exception e) {}

        motor.setPower(0);

        pos[1] = motor.getCurrentPosition();

        telemetry.addData("Value Backward", pos[1]);
        telemetry.update();

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       if (pos[0] > 200 && pos[1] < -200) motorWorking = true;

       return motorWorking;

    }

}
