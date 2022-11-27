package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotMain;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ScoringMechanism extends Subsystem {

    private static final Subsystem instance = new ScoringMechanism();

    public DcMotor intake;
    public DcMotor slide;
    public Servo basket;
    public DcMotor duckArm;
    public Servo capping;
    public DistanceSensor distance;
    public ColorSensor color;
   // public CRServo capping;

    private final double intakePower = 1;
    private final double slidePower = -0.8; //-1
    private final double duckArmPower = 1;
    private final double cappingPower = 1;

    private ElapsedTime timer = new ElapsedTime();

    private ScoringMechanism() {
        //Private constructor
    }

    @Override
    public void subsystemInit(HardwareMap hardwareMap) {
        //Init motors
        intake = hardwareMap.get(DcMotor.class, "intake");
        slide = hardwareMap.get(DcMotor.class, "slide");
        basket = hardwareMap.get(Servo.class, "basket");
        duckArm = hardwareMap.get(DcMotor.class, "duckArm");
        capping = hardwareMap.get(Servo.class, "capping");
       // capping = hardwareMap.get(CRServo.class, "capping");
        distance = hardwareMap.get(DistanceSensor.class,"distance");
        color = hardwareMap.get(ColorSensor.class, "color");

        //Set motors to break
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        timer.reset();

        basket.setPosition(0.4);
        capping.setPosition(0.15);
    }

    @Override
    public void loop() {
        if (RobotMain.gamepad2.dpad_up) {
            slide.setPower(RobotMain.gamepad2.right_bumper ? slidePower / 2 : slidePower);
        } else if (RobotMain.gamepad2.dpad_down) {
            slide.setPower(RobotMain.gamepad2.right_bumper ? -slidePower/2 : -slidePower);
        } else if (RobotMain.gamepad2.left_stick_y >0){
            intake(intakePower);
        } else if (RobotMain.gamepad2.left_stick_y <0){
            intake(-intakePower);
        } else if (RobotMain.gamepad2.y) {
            basket.setPosition(0.85);
        } else if (RobotMain.gamepad2.a) {
            basket.setPosition(0);
        } else if (RobotMain.gamepad2.b)  {
           duckArm.setPower(duckArmPower);
        }
        else if (RobotMain.gamepad2.right_stick_y != 0) {
            duckArm.setPower(-duckArmPower * RobotMain.gamepad2.right_stick_y);
        } else if (RobotMain.gamepad1.dpad_up) {
            cappingMotion(1);
        } else if (RobotMain.gamepad1.dpad_down) {
            cappingMotion(-1);
        }
         else {
            slide.setPower(0);
            intake.setPower(0);
            basket.setPosition(0);
            duckArm.setPower(0);
          //  capping.setPower(0);
        }
    }

    @Override
    public void stop() {
        slide.setPower(0);
        intake.setPower(0);
        duckArm.setPower(0);
    }

    public void slideByTicks(double power, double ticks) {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int setPoint = (int) ticks;
        slide.setPower(power);
        slide.setTargetPosition(setPoint);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // TODO replace timer with encoder position-based loop
        while (slide.getCurrentPosition() < (ticks - 10) || slide.getCurrentPosition() > (ticks + 10)) {
            //changed to slide.getpos instead of timer (not as accurate)
            //more likely to reach its position more consistently
        }
        slide.setPower(0);
    }

    public void cappingMotion(double power) {
//     capping.setPower(power);
 }
    public void basketMotion() {
        basket.setPosition(0.85);

        timer.reset();
        while (timer.milliseconds() < 1800) {
            // do nothing
        }

        basket.setPosition(0);
    }

    public void intake(double power) {
        intake.setPower(power);
    }

    public void slide(double power) { slide.setPower(power); }

    public void duckArm(double power) { duckArm.setPower(power); }

    /**
     * @return the singleton instance of the subsystem
     */
    public static Subsystem getInstance() {
        return instance;
    }
}

