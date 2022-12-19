package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.ScoringMechanism;


@Autonomous(name="BlueRightVisionConePark", group="Linear Opmode")

public class BlueRightVisionConePark extends LinearOpMode {

    private RobotMain robot;
    private DriveTrain driveTrain;
    private ScoringMechanism scoringMechanism;
    private ElapsedTime timer = new ElapsedTime();
    private int parkPos;

    @Override
    public void runOpMode() {
        robot = new RobotMain(hardwareMap, gamepad1, gamepad2, "blue", true, telemetry);
        driveTrain = (DriveTrain) RobotMain.driveTrain;
        scoringMechanism = (ScoringMechanism) RobotMain.scoringMechanism;
        timer.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!isStarted()) {
            parkPos = robot.coneTagDetector.getTagNumber(telemetry);
            sleep(20);
        }

        // code to run sequentially for 30 seconds
        if (opModeIsActive()) {

            if (parkPos==17) {

            } else if (parkPos==18) {

            } else if (parkPos==19) {

            } else {

            }


        }


    }
}

