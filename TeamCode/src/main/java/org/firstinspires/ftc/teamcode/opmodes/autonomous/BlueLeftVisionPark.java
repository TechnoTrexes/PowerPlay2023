package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.ScoringMechanism;


@Autonomous(name="BlueLeftVisionPark", group="Linear Opmode")

public class BlueLeftVisionPark extends LinearOpMode {

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
            scoringMechanism.closeGripper();
            scoringMechanism.slideByTicks(0.2, -100);
            driveTrain.driveDistance(0.8, 26, 270, false);
            driveTrain.driveDistance(0.8, 62, 0, false);

            if (parkPos==17) {
                driveTrain.driveDistance(0.8,1,0,false);
            } else if (parkPos==18) {
                driveTrain.driveDistance(0.8,24,90,false);
            } else if (parkPos==19) {
                driveTrain.driveDistance(0.8,48,90,false);
            } else {

            }


        }


    }
}

