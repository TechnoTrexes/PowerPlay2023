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
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.ScoringMechanism;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp(name="TeleopSide", group="Iterative Opmode")

public class TeleopSide extends OpMode {

    private RobotMain robot;
    private DriveTrain driveTrain;
    private ScoringMechanism scoringMechanism;

    @Override
    public void init() {
        robot = new RobotMain(hardwareMap, gamepad1, gamepad2, "blue", true);
        driveTrain = (DriveTrain) RobotMain.driveTrain;
        scoringMechanism = (ScoringMechanism) RobotMain.scoringMechanism;

        scoringMechanism.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scoringMechanism.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double d= 0;
        d= scoringMechanism.distance.getDistance(DistanceUnit.MM);
        telemetry.addData("distance= ", d);

        telemetry.addData("Red", scoringMechanism.color.red());
        telemetry.addData("Green", scoringMechanism.color.green());
        telemetry.addData("Blue", scoringMechanism.color.blue());
        telemetry.addData("alpha", scoringMechanism.color.alpha());
        telemetry.addData("argb", scoringMechanism.color.argb());


        telemetry.update();
        int pos = scoringMechanism.slide.getCurrentPosition();
        if (gamepad2.dpad_up) {
            scoringMechanism.slide.setPower(-0.05);
        } else if (gamepad2.dpad_down) {
            scoringMechanism.slide.setPower(0.05);
        } else if (gamepad2.b) { //close
            scoringMechanism.basket.setPosition(1);
            scoringMechanism.capping.setPosition(0);
        } else if (gamepad2.x) { //open
            scoringMechanism.basket.setPosition(0.4);
            scoringMechanism.capping.setPosition(0);
        } else if (RobotMain.gamepad2.x) {
            scoringMechanism.capping.setPosition(0.5);
        } else if (RobotMain.gamepad2.b) {
            scoringMechanism.capping.setPosition(0); // close to cup
            telemetry.addData("slide motor position:", pos);
            telemetry.update();
        } else {
            scoringMechanism.slide.setPower(0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //test change
    }
}
