package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.ScoringMechanism;


@Autonomous(name="RedRightVisionConePark", group="Linear Opmode")

public class RedRightVisionConePark extends LinearOpMode {

    private RobotMain robot;
    private DriveTrain driveTrain;
    private ScoringMechanism scoringMechanism;
    private ElapsedTime timer = new ElapsedTime();
    private int parkPos;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMain(hardwareMap, gamepad1, gamepad2, "red", true, telemetry);
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
            driveTrain.driveDistance(0.8, 21, 90, false); // was 24
            scoringMechanism.slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            scoringMechanism.slideByTicks(0.3, -4200); //4200
            scoringMechanism.slide.setPower(0.01);
            driveTrain.driveDistance(0.8, 76, 0, false); //
            //driveTrain.rotateDegrees(0.8,-90);
            driveTrain.driveDistance(0.2, 2, 270, false); // 4
            Thread.sleep(2000);
            scoringMechanism.slideByTicks(0.3,650);
            scoringMechanism.openGripper();
            driveTrain.driveDistance(0.8,4,90,false);
            driveTrain.driveDistance(0.8,14,180,false);
            if (parkPos==17) {
            driveTrain.driveDistance(0.8,1,180,false);
            } else if (parkPos==18) {
                driveTrain.driveDistance(0.8,24,270,false);
            } else if (parkPos==19) {
                driveTrain.driveDistance(0.8,48,270,false);
            } else {

            }
            scoringMechanism.slide.setPower(0);

        }


    }
}

