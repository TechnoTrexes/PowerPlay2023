//test commit

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.OpenCV.ConeTagDetector;
import org.firstinspires.ftc.teamcode.libs.drive.Pose2d;
import org.firstinspires.ftc.teamcode.libs.vision.Ducktector;
import org.firstinspires.ftc.teamcode.subsystems.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//distance sensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class RobotMain {

    //Declare hardwareMap
    public static HardwareMap hardwareMap;

    //Declare gamepads
    public static Gamepad gamepad1;
    public static Gamepad gamepad2;

    //Declare all & put in ArrayList
    public static Subsystem driveTrain = DriveTrain.getInstance();
    public static Subsystem scoringMechanism = ScoringMechanism.getInstance();
    public static Subsystem[] allSubsystems = {driveTrain, scoringMechanism};

    //Declare eemuu (gyroscope)
    public static BNO055IMU imu;

    //Misc
    public String allianceColor;

    //Declare vision members
    public ConeTagDetector coneTagDetector;
    public static int parkPos;

    Telemetry telemetry;
    //Declare misc objects
    private ElapsedTime timer;
    public static boolean godtonomous;

    public RobotMain(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, String allianceColor, boolean godtonomous) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.allianceColor = allianceColor.toLowerCase();
        robotInit(godtonomous);
    }

    public RobotMain(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, String allianceColor,
                     boolean godtonomous, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.allianceColor = allianceColor.toLowerCase();
        this.telemetry = telemetry;
        robotInit(godtonomous);
    }

    private void robotInit(boolean godtonomous) {
        //Init all subsystems
        for (Subsystem subsystem : allSubsystems) {
            subsystem.subsystemInit(hardwareMap);
        }

        //Init eemuu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        
        //Init vision
        if (godtonomous) {
            //Init vision
            coneTagDetector = new ConeTagDetector(hardwareMap);
            
            //Detect for five seconds to get an accurate reading
            timer = new ElapsedTime();
            timer.reset();
            while (timer.seconds() < 3) {
                parkPos = coneTagDetector.getTagNumber(telemetry);
            }

            //Shut the camera off before init is complete for power conservation
            //phoneCam.closeCameraDevice();
        }
    }

    //For if regular framework stops working
    public DcMotor getDcMotor(String name) { return hardwareMap.get(DcMotor.class, name); }

    public Servo getServoMotor(String name) { return hardwareMap.get(Servo.class, name); }

    public CRServo getCRServoMotor(String name) { return hardwareMap.get(CRServo.class, name); }

    //Gyro control
    /*public static double getAngle() {
        //Retrieve raw angle
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double rawAngle = angles.firstAngle;

        //Manipulate angle to unit circle convention (degrees)
        if (rawAngle < 0) {
            rawAngle = 360 + rawAngle;
        }

        //Orient to unit circle convention (degrees)
        rawAngle -= 90;
        if (rawAngle < 0) {
            rawAngle += 360;
        }

        return rawAngle;
    }*/
}
