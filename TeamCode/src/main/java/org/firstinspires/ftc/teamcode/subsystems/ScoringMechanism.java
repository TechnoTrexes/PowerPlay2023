package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
  //  public ColorSensor color;
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
       // color = hardwareMap.get(ColorSensor.class, "color");

        //Set motors to break
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        timer.reset();

        basket.setPosition(0.35);
        capping.setPosition(0.10);
    }

    @Override
    public void loop(Telemetry telemetry) {
        double d= 0;
        d= distance.getDistance(DistanceUnit.MM);
       telemetry.addData("distance= ", d);

     //   telemetry.addData("Red", color.red());
        //telemetry.addData("Green", color.green());
        //telemetry.addData("Blue", color.blue());
        //telemetry.addData("alpha", color.alpha());
        //telemetry.addData("argb", color.argb());


        //telemetry.update();
        int pos = slide.getCurrentPosition();
        if (RobotMain.gamepad2.dpad_up) {
            slide.setPower(-0.5);
        } else if (RobotMain.gamepad2.dpad_down) {
            slide.setPower(0.5);
        } else if (RobotMain.gamepad2.b) { //close
         //   basket.setPosition(1);
           // capping.setPosition(0);
            closeGripper();
        } else if (RobotMain.gamepad2.x) { //open
          //  basket.setPosition(0.4);
           // capping.setPosition(0.15);
            openGripper();
        } else if (RobotMain.gamepad2.x) {
            capping.setPosition(0.5);
        } else if (RobotMain.gamepad2.b) {
            capping.setPosition(0); // close to cup
          //  telemetry.addData("slide motor position:", pos);
           // telemetry.update();
        }
         else {
            slide.setPower(0);

    //   intake.setPower(0);
//            duckArm.setPower(0);
           // basket.setPosition(1);
            //capping.setPosition(0);

            //  capping.setPower(0);
        }
    }

    @Override
    public void stop() {
        slide.setPower(0);
      //  intake.setPower(0);
      //  duckArm.setPower(0);
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
    public void closeGripper() {
        basket.setPosition(1); //1
        capping.setPosition (0); //0
    }
    public void openGripper() {
        basket.setPosition(0.4);
        capping.setPosition(0.15);
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

