package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "webCamTest")
public class webcamTest extends LinearOpMode {

  Hardware robot = new Hardware();

  public void runOpMode() {

      robot.init(hardwareMap);

      robot.initVuforia();

     //hitestlolalex


      waitForStart();

      while (opModeIsActive()) {
          robot.lookForStone();

          telemetry.addData("See target?", robot.targetVisible);
          telemetry.addData("Distance X", robot.robotDistanceToStone[0]);
          telemetry.addData("Distance Y", robot.robotDistanceToStone[1]);
          telemetry.update();
      }

  }





}
