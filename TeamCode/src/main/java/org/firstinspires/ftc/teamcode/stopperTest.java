package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "stopperTest")
public class stopperTest extends LinearOpMode {

    Hardware robot = new Hardware();

    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            if (-gamepad1.right_stick_y > 0) robot.armMotor.setPower(0.5);


            if (-gamepad1.right_stick_y < 0) robot.armMotor.setPower(-0.5);


            if (-gamepad1.right_stick_y == 0) robot.armMotor.setPower(0);

            if (robot.armMotor.getPower() == 0) {
                //get actual value
                robot.armStopServo.setPosition(1);
            } else {
                //get actual value
                robot.armStopServo.setPosition(0);
            }

        }



    }




}
