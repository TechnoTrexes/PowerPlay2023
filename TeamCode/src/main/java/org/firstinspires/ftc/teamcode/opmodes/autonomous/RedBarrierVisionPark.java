/*
Copyright 2019 FIRST Tech Challenge Team 12923

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.subsystems.*;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Autonomous(name="RedBarrierVisionPark", group="Linear Opmode")

public class RedBarrierVisionPark extends LinearOpMode {

    private RobotMain robot;
    private DriveTrain driveTrain;
    private ScoringMechanism scoringMechanism;

    private char duckPos;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot = new RobotMain(hardwareMap, gamepad1, gamepad2, "red", true);
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

            driveTrain.driveDistance(0.6, 10, 270, false, telemetry); // was 4
            driveTrain.driveDistance(0.6, 28, 0, false);  // 24
            driveTrain.driveDistance(0.6, 13, 270, false); // 15
            if (duckPos=='L') {
                scoringMechanism.slideByTicks(0.3, Constants.BOTTOM_TIER_TICKS);
            } else if (duckPos=='M') {
                scoringMechanism.slideByTicks(0.3, Constants.MIDDLE_TIER_TICKS);
            } else if (duckPos=='R') {
                scoringMechanism.slideByTicks(0.3, Constants.TOP_TIER_TICKS);
            } else {
                scoringMechanism.slideByTicks(0.3, Constants.TOP_TIER_TICKS);
                //arm goes to top (r)
            }
            scoringMechanism.basketMotion();


            driveTrain.driveDistance(0.6, 4, 90, false);
            driveTrain.driveDistance(0.8, 3, 90, false);
            driveTrain.rotateDegrees(0.6, -600); // 90
            scoringMechanism.slideByTicks(0.8, -Constants.BOTTOM_TIER_TICKS);
            driveTrain.rotateDegrees(0.8,1150);
            driveTrain.driveDistance(0.8,40,0,false);
            driveTrain.driveDistance(1, 55, 270, false);
        }


    }
}

