package org.firstinspires.ftc.teamcode.opmodes.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.subsystems.*;

import org.firstinspires.ftc.teamcode.Constants;

@Autonomous(name="RedStorageVisionDuck", group="Linear Opmode")

public class RedStorageVisionDuck extends LinearOpMode {

    private RobotMain robot;
    private DriveTrain driveTrain;
    private ScoringMechanism scoringMechanism;

    private char duckPos;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot = new RobotMain(hardwareMap, gamepad1, gamepad2, "blue", true);
        driveTrain = (DriveTrain) RobotMain.driveTrain;
        scoringMechanism = (ScoringMechanism) RobotMain.scoringMechanism;
        timer.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // code to run repeatedly after 'init' is pressed but before 'start' is pressed
        while (!isStarted()){
            duckPos = robot.ducktector.getDuckPosition();
            telemetry.addData("Duck Position", duckPos);
            telemetry.update();
        }

        /* leave this loop for quick testing
        if (opModeIsActive()){
            driveTrain.rotateDegrees(0.3, 600);
        }
        */


        // code to run sequentially for 30 seconds
        if (opModeIsActive()) {

            timer.reset();
            while (timer.milliseconds() < 7500) {}
                // do nothing
            driveTrain.driveDistance(0.8, 10, 270, false); // was 4
            driveTrain.driveDistance(0.8,25,180,false);  // driveTrain.driveDistance(0.8, 37, 180, false); // 24
            int distance=12;
            if (duckPos=='R') {
                distance=16-Constants.BOTTOM_TIER_DISTANCEDIFF;
            }
            driveTrain.driveDistance(0.8, distance, 270, false); // 13
            if (duckPos=='R') {
                scoringMechanism.slideByTicks(0.8, Constants.BOTTOM_TIER_TICKS);
            } else if (duckPos=='M') {
                scoringMechanism.slideByTicks(0.8, Constants.MIDDLE_TIER_TICKS);
            } else if (duckPos=='L') {
                scoringMechanism.slideByTicks(0.8, Constants.TOP_TIER_TICKS);
            } else {
                scoringMechanism.slideByTicks(0.8, Constants.TOP_TIER_TICKS);
                //arm goes to top (r)
            }
           scoringMechanism.basketMotion();


            driveTrain.driveDistance(0.8, 3, 90, false);
            driveTrain.rotateDegrees(0.8, -1150);
            scoringMechanism.slideByTicks(0.8, -Constants.BOTTOM_TIER_TICKS);
            if (duckPos=='R') {
                driveTrain.driveDistance(0.8, 23 -Constants.BOTTOM_TIER_DISTANCEDIFF , 270, false);
            }else {
                driveTrain.driveDistance(0.8, 19, 270, false);
            }
            driveTrain.driveDistance(0.8, 57, 180, false,true);
            scoringMechanism.duckArm(1);
            timer.reset();
            while (timer.milliseconds() < 4000) {
                // do nothing
            }
            driveTrain.driveDistance(1, 27,90, false);
            driveTrain.driveDistance(1, 3,180, false);


        }

    }

}

