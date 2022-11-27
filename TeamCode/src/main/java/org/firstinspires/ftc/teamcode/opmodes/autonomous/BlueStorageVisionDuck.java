package org.firstinspires.ftc.teamcode.opmodes.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.subsystems.*;

import org.firstinspires.ftc.teamcode.Constants;

@Autonomous(name="BlueStorageVisionDuck", group="Linear Opmode")

public class BlueStorageVisionDuck extends LinearOpMode {

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
            while (timer.milliseconds() < 6000)
            {}
            driveTrain.driveDistance(0.8, 10, 270, false); // was 4
            driveTrain.driveDistance(0.8, 35, 0, false); // 24
            int distance=12;
            if (duckPos=='L') {
                distance= 12-Constants.BOTTOM_TIER_DISTANCEDIFF;
            }
            driveTrain.driveDistance(0.8, distance, 270, false); // 13
            if (duckPos=='L') {
                scoringMechanism.slideByTicks(0.8, Constants.BOTTOM_TIER_TICKS);
            } else if (duckPos=='M') {
                scoringMechanism.slideByTicks(0.8, Constants.MIDDLE_TIER_TICKS);
            } else if (duckPos=='R') {
                scoringMechanism.slideByTicks(0.8, Constants.TOP_TIER_TICKS);
            } else {
                scoringMechanism.slideByTicks(0.8, Constants.TOP_TIER_TICKS);
                //arm goes to top (r)
            }
            scoringMechanism.basketMotion();


            driveTrain.driveDistance(0.8, 3, 90, false);
            driveTrain.rotateDegrees(0.8, 560); // 90
            scoringMechanism.slideByTicks(0.8, -Constants.BOTTOM_TIER_TICKS);
            driveTrain.driveDistance(0.8, 10, 180, false);
            driveTrain.driveDistance(0.8, 50, 270, false);
            driveTrain.driveDistance(0.8, 7, 180, false, true); //straffing before duck (16 og)
            driveTrain.driveDistance(0.5, 3, 270, false);
            scoringMechanism.duckArm(-0.7);
            timer.reset();
            while (timer.milliseconds() < 4000) {
                // do nothing
           }
            driveTrain.driveDistance(1, 24,0, false);
           //driveTrain.driveDistance(1, 4,270, false);

        }

    }

}

