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

    final double tolerance = 6; //TODO tune # of millimeters

    private boolean lockedOnTarget = false;
    private final int LEVEL1_MM = 500; //TODO
    private final int LEVEL2_MM = 735; //TODO
    private final int LEVEL3_MM = 980; //TODO
    private double goalHeight;

    //  public ColorSensor color;

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
        goalHeight = distance.getDistance(DistanceUnit.MM);
    }

    @Override
    public void loop(Telemetry telemetry) {
        double d = distance.getDistance(DistanceUnit.MM);
        telemetry.addData("lockedOntoTarget", lockedOnTarget);
        telemetry.update();

        //If not locked onto a target, prevent slide from going down on its own weight
        if (!lockedOnTarget && Math.abs(RobotMain.gamepad2.left_stick_y) < 0.1) {
            stayAtCurrentHeight();
        }

        //Auto go to level (or set power to 0 if slideToLevel() motion is done)
        if (slideToLevel() && lockedOnTarget) {
            slide.setPower(0);
        }
        telemetry.addData("goal height", goalHeight);

        int pos = slide.getCurrentPosition();
        if (RobotMain.gamepad2.dpad_down) {
            setGoalHeight(1, false); //false doesn't matter here
        } else if (RobotMain.gamepad2.dpad_right) {
            setGoalHeight(2, false); //false doesn't matter here
        } else if (RobotMain.gamepad2.dpad_up) {
            setGoalHeight(3, false); //false doesn't matter here
        } else if (RobotMain.gamepad2.left_stick_y < -0.1) {
            setGoalHeight(0, false); //Stop auto locking onto a position
            if (d > 750) {
                slide.setPower(-0.25);
            } else {
                slide.setPower(-0.5);
            }
        } else if (RobotMain.gamepad2.left_stick_y > 0.1) {
            setGoalHeight(0, true); //Stop auto locking onto a position
            if (digitalTouch.getState()) {
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

    /**
     * Sets the goal height either to a level (1, 2, or 3), or to the current position for manual control
     * @param level 1, 2, or 3, or 0 for manual control
     * @param direction doesn't matter in most cases, but true if it's going up, false if slide is currently
     *                  going down
     */
    public void setGoalHeight(int level, boolean direction) {
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
                //(direction ? - 1 : 1) just expands to:
                //if (direction == true) {
                //  return -1;
                // } else {
                //  return 1;
                //}

                //That will multiply tolerance by -1 if direction is true, or 1 if it's false
                goalHeight = distance.getDistance(DistanceUnit.MM) + tolerance * (direction ? - 1 : 1);
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
        final double kP = -0.005; //TODO tune

        double dd = distance.getDistance(DistanceUnit.MM);
        if (dd < 75.0) {
            // do nothing
            return true;
        }
        if (lockedOnTarget) {
            double error = (goalHeight - dd);

            if (Math.abs(error) > tolerance) {
                slide.setPower(error * kP);
                return false;
            } else {
                slide.setPower(0);
            }
        }
        return true;
    }

    @Override
    public void stop() {
        slide.setPower(0);
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

    /**
     * @return the singleton instance of the subsystem
     */
    public static Subsystem getInstance() {
        return instance;
    }
}

