package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    public DigitalChannel digitalTouch;  // Hardware Device Object

    private boolean lockedOnTarget = false;
    private final int LEVEL1_MM = 500; //TODO
    private final int LEVEL2_MM = 735; //TODO
    private final int LEVEL3_MM = 980; //TODO
    private int goalHeight;

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
        distance = hardwareMap.get(DistanceSensor.class,"distance");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "touchSensor");

        //Set motors to break
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        basket.setPosition(0.35);
        capping.setPosition(0.65);
        timer.reset();

    }

    @Override
    public void loop(Telemetry telemetry) {
        double d = distance.getDistance(DistanceUnit.MM);
        telemetry.addData("distance= ", d);
        telemetry.update();

        //If not locked onto a target, prevent slide from going down on its own weight
        if (!lockedOnTarget && !RobotMain.gamepad2.dpad_down && !RobotMain.gamepad2.dpad_up) {
            stayAtCurrentHeight();
        }

        //Auto go to level (or set power to 0 if slideToLevel() motion is done)
        if (slideToLevel()) {
            slide.setPower(0);
        }

        int pos = slide.getCurrentPosition();
        if (RobotMain.gamepad2.y) {
            setGoalHeight(1);
            //telemetry.addData("goalheight= ", goalHeight);
            //telemetry.update();
        } else if (RobotMain.gamepad2.a) {
            setGoalHeight(2);
        } else if (RobotMain.gamepad2.dpad_right) {
            setGoalHeight(3);
        } else if (RobotMain.gamepad2.dpad_up) {
            setGoalHeight(0); //Stop auto locking onto a position
            if (d > 750) {
                slide.setPower(-0.25);
            } else {
                slide.setPower(-0.5);
            }
        } else if (RobotMain.gamepad2.dpad_down) {
            setGoalHeight(0); //Stop auto locking onto a position
            if (digitalTouch.getState() == true) {
                slide.setPower(0.5);
            } else {
                slide.setPower(0);
            }

        } else if (RobotMain.gamepad2.b) { //close
            closeGripper();
        } else if (RobotMain.gamepad2.x) { //open
            openGripper();
            //  telemetry.addData("slide motor position:", pos);
            // telemetry.update();
        }
    }

    public void setGoalHeight(int level) {
        switch (level) {
            case 1: 
                goalHeight = LEVEL1_MM;
                lockedOnTarget = true;
                break;
            case 2: 
                goalHeight = LEVEL2_MM;
                lockedOnTarget = true;
                break;
            case 3: 
                goalHeight = LEVEL3_MM;
                lockedOnTarget = true;
                break;
            default: 
                goalHeight = distance.getDistance();
                lockedOnTarget = false;
                break;
        }
    }

    /**
     * Updates slider speed based off of dist from ground but ONLY when 
     * the slider isn't moving (i.e. when the drivers aren't pressing up or down)
     */
    public void stayAtCurrentHeight() {
        //Very janky hack so that we don't have to copy paste the slideToLevel() code:
        //set lockedOnTarget to true ONLY for this iteration, then we set it back to false
        //since we're guaranteed that lockedOnTarget will be false once entering this method
        //and we want it to be in the same state leaving it
        lockedOnTarget = true;
        slideToLevel();
        lockedOnTarget = false;
    }

    /**
     * Updates slider speed based off of dist from ground
     * @return whether or not motion is finished
     */
    public boolean slideToLevel() {
        if (lockedOnTarget) {
            final double kP = -0.005; //TODO tune
            final double tolerance = 10; //TODO tune # of millimeters
            double error = (goalHeight - distance.getDistance(DistanceUnit.MM));

            if (Math.abs(error) > tolerance) {
                slide.setPower(error * kP);
                return false;
            }
            else {
                slide.setPower(0);
            }
        }
        return true;
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

        //TODO uncomment when ready
        // while (!updateSliderSpeed()) {}
        slide.setPower(0);
    }
    public void closeGripper() {
        basket.setPosition(1); //1
        capping.setPosition (0); //0
    }
    public void openGripper() {
        basket.setPosition(0.35);
        capping.setPosition(0.65);
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

