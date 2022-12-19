package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMain;
import org.firstinspires.ftc.teamcode.libs.drive.PIDController;

public class DriveTrain extends Subsystem {

    //Declare private instance
    private static final Subsystem instance = new DriveTrain();

    //Declare motors
    public DcMotorEx topLeft;
    public DcMotorEx bottomLeft;
    public DcMotorEx topRight;
    public DcMotorEx bottomRight;

    //Declare encoders
    public DcMotor leftEncoder;
    public DcMotor rightEncoder;
    public DcMotor horizontalEncoder;

    //Declare constants
    private static final double VIDIPT_DRIVE_CONTROL = 0.7; //1 is og
    //private double VIDIPT_DRIVE_CONTROL = 1;
    private static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    private static final int TICKS_PER_ROTATION = 538;
    private static final double GEAR_RATIO = 1;
    private static final double ENCODER_TOLERANCE = 20;

    //Declare PID members
    private PIDCoefficients PIDcoeffs;
    private PIDController PIDController;

    //Misc
    private ElapsedTime timer;

    //Private constructor
    private DriveTrain() {
        timer = new ElapsedTime();
    }

    @Override
    public void subsystemInit(HardwareMap hardwareMap) {
        //Init motors
        topLeft = hardwareMap.get(DcMotorEx.class, "topLeft");
        bottomLeft = hardwareMap.get(DcMotorEx.class, "bottomLeft");
        topRight = hardwareMap.get(DcMotorEx.class, "topRight");
        bottomRight = hardwareMap.get(DcMotorEx.class, "bottomRight");

        //Init encoders
        leftEncoder = hardwareMap.get(DcMotor.class, "topLeft");
        rightEncoder = hardwareMap.get(DcMotor.class, "topRight");
        horizontalEncoder = hardwareMap.get(DcMotor.class, "bottomLeft");

        //Set motors to break
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reverse left side motors
        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Enable & reset encoders
        topLeft.setTargetPositionTolerance(20);
        bottomLeft.setTargetPositionTolerance(20);
        topRight.setTargetPositionTolerance(20);
        bottomRight.setTargetPositionTolerance(20);
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Reset odometry encoders
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set mode of odometry encoders to RUN_WITHOUT_ENCODER
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Reverse odometry encoders as needed here:
        //____Encoder.setDirection(DcMotorSimple.Direction.REVERSE)

        //Init PID members
        PIDcoeffs = new PIDCoefficients(0, 0, 0);
        DcMotor[] motors = {topLeft, bottomLeft, topRight, bottomRight};
        PIDController = new PIDController(motors, PIDcoeffs, 6);
    }

    @Override
    public void loop(Telemetry telemetry) {
        double thrust = RobotMain.gamepad1.left_stick_y; //originally negative
        double strafe = -RobotMain.gamepad1.left_stick_x; //originally positive
        double turn = -RobotMain.gamepad1.right_stick_x; //originally positive
        driveMecanum(thrust, strafe, turn, false);
/*
        if (RobotMain.gamepad1.dpad_up) {
            rotateTo(90);
        }
 */
    }

    @Override
    public void stop() {

    }

    /**
     * Basic tank drive that controls left wheels and right wheels independently
     *
     * @param leftPower power applied to left wheels
     * @param rightPower power applied to right wheels
     */
    public void driveTank(double leftPower, double rightPower) {
        topLeft.setPower(leftPower * VIDIPT_DRIVE_CONTROL);
        topRight.setPower(rightPower * VIDIPT_DRIVE_CONTROL);
        bottomLeft.setPower(leftPower * VIDIPT_DRIVE_CONTROL);
        bottomRight.setPower(rightPower * VIDIPT_DRIVE_CONTROL);
    }

    /**
     * Determines mecanum wheel powers with given components. Feed in joystick values to drive in
     * desired direction.
     *
     * @param thrust y component of requested vector
     * @param strafe x component of requested vector
     * @param turn turn component (z axis) of robot while traveling along requested vector
     * @param fieldCentric whether robot should move relative to field or drivers
     *                     (true = relative to field; false = relative to drivers)
     */
    public void driveMecanum(double thrust, double strafe, double turn, boolean fieldCentric) {
        //If field centric -- does nothing yet
        if (fieldCentric) {
            thrust = thrust;
            strafe = strafe;
        }

        if (Math.hypot(thrust, strafe) <= 0.5) {
            thrust *= 0.5;
            strafe *= 0.5;
        }
        if (Math.abs(turn) <= 0.5) {
            turn *= 0.5;
        }

        //Determining wheel powers
        double topLeftPower = thrust + strafe + turn;
        double bottomLeftPower = thrust - strafe + turn;
        double topRightPower = thrust - strafe - turn;
        double bottomRightPower = thrust + strafe - turn;

        //Scale based off max power
        double maxPower = Math.max(Math.max(Math.max(topLeftPower, bottomLeftPower), topRightPower), bottomRightPower);
        if (Math.abs(maxPower) > 1.0) {
            topLeftPower /= Math.abs(maxPower);
            bottomLeftPower /= Math.abs(maxPower);
            topRightPower /= Math.abs(maxPower);
            bottomRightPower /= Math.abs(maxPower);
        }

        //Vidipt control
        topLeftPower *= VIDIPT_DRIVE_CONTROL;
        bottomLeftPower *= VIDIPT_DRIVE_CONTROL;
        topRightPower *= VIDIPT_DRIVE_CONTROL;
        bottomRightPower *= VIDIPT_DRIVE_CONTROL;

        //Set powers
        topLeft.setPower(topLeftPower);
        topRight.setPower(topRightPower);
        bottomLeft.setPower(bottomLeftPower);
        bottomRight.setPower(bottomRightPower);
    }

    /**
     * Drives a certain distance in a forward/backward/left/right direction using encoders
     *
     * @param power power applied to all motors
     * @param inches distance traveled by each wheel of drivetrain
     * @param degreeDirection direction requested to travel by robot
     *                        0, 90, 180, and 270 will travel distance;
     *                        any other angle will not do anything.
     * @param PID whether or not to use PID
     */
    public void driveDistance(double power, int inches, double degreeDirection, boolean PID) {
        //Only F, B, L, and R directions
        if (degreeDirection % 90 != 0) {
            return;
        }

        //Determine component values
        double thrust = Math.sin(Math.toRadians(degreeDirection)) * power;
        double strafe = Math.cos(Math.toRadians(degreeDirection)) * power;

        //Ready encoders & set target positions (TLBR = topLeft & bottomRight motors; BLTR = bottomLeft & topRight motors)
        int setPoint = toTicks(inches);
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int TLBRSetpt = thrust + strafe > 0 ? setPoint : -setPoint;
        int BLTRSetpt = thrust - strafe > 0 ? setPoint : -setPoint;

        boolean[] motorDirections = new boolean[4];
        if (TLBRSetpt < 0) {
            motorDirections[0] = true;
            motorDirections[3] = true;
        }
        if (BLTRSetpt < 0) {
            motorDirections[1] = true;
            motorDirections[2] = true;
        }

        topLeft.setTargetPosition(TLBRSetpt);
        bottomLeft.setTargetPosition(BLTRSetpt);
        topRight.setTargetPosition(BLTRSetpt);
        bottomRight.setTargetPosition(TLBRSetpt);


        setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (PID) {
            PIDController.drive(setPoint, ENCODER_TOLERANCE, motorDirections);
        } else {
            driveMecanum(thrust, strafe, 0, false);
            while (topLeft.isBusy() || bottomLeft.isBusy() || topRight.isBusy() || bottomRight.isBusy()) {
                //robot moves
            }
            driveTank(0, 0);
        }
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveDistance(double power, int inches, double degreeDirection, boolean PID, Telemetry telemetry) {
        //Only F, B, L, and R directions
        if (degreeDirection % 90 != 0) {
            return;
        }

        //Determine component values
        double thrust = Math.sin(Math.toRadians(degreeDirection)) * power;
        double strafe = Math.cos(Math.toRadians(degreeDirection)) * power;

        //Ready encoders & set target positions (TLBR = topLeft & bottomRight motors; BLTR = bottomLeft & topRight motors)
        int setPoint = toTicks(inches);
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int TLBRSetpt = thrust + strafe > 0 ? setPoint : -setPoint;
        int BLTRSetpt = thrust - strafe > 0 ? setPoint : -setPoint;

        boolean[] motorDirections = new boolean[4];
        if (TLBRSetpt < 0) {
            motorDirections[0] = true;
            motorDirections[3] = true;
        }
        if (BLTRSetpt < 0) {
            motorDirections[1] = true;
            motorDirections[2] = true;
        }

        topLeft.setTargetPosition(TLBRSetpt);
        bottomLeft.setTargetPosition(BLTRSetpt);
        topRight.setTargetPosition(BLTRSetpt);
        bottomRight.setTargetPosition(TLBRSetpt);


        setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (PID) {
            PIDController.drive(setPoint, ENCODER_TOLERANCE, motorDirections);
        } else {
            driveMecanum(thrust, strafe, 0, false);
            while (topLeft.isBusy() || bottomLeft.isBusy() || topRight.isBusy() || bottomRight.isBusy()) {
                telemetry.addData("topLeft", topLeft.isBusy());
                telemetry.addData("bottomLeft", bottomLeft.isBusy());
                telemetry.addData("topRight", topRight.isBusy());
                telemetry.addData("bottomRight", bottomRight.isBusy());
                telemetry.update();
            }
            driveTank(0, 0);
        }
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveDistance(double power, int inches, double degreeDirection, boolean PID, boolean noAdjust) {
        //Only F, B, L, and R directions
        if (degreeDirection % 90 != 0) {
            return;
        }

        //Determine component values
        double thrust = Math.sin(Math.toRadians(degreeDirection)) * power;
        double strafe = Math.cos(Math.toRadians(degreeDirection)) * power;

        //Ready encoders & set target positions (TLBR = topLeft & bottomRight motors; BLTR = bottomLeft & topRight motors)
        int setPoint = toTicks(inches);
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int TLBRSetpt = thrust + strafe > 0 ? setPoint : -setPoint;
        int BLTRSetpt = thrust - strafe > 0 ? setPoint : -setPoint;

        boolean[] motorDirections = new boolean[4];
        if (TLBRSetpt < 0) {
            motorDirections[0] = true;
            motorDirections[3] = true;
        }
        if (BLTRSetpt < 0) {
            motorDirections[1] = true;
            motorDirections[2] = true;
        }

        topLeft.setTargetPosition(TLBRSetpt);
        bottomLeft.setTargetPosition(BLTRSetpt);
        topRight.setTargetPosition(BLTRSetpt);
        bottomRight.setTargetPosition(TLBRSetpt);


        setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (PID) {
            PIDController.drive(setPoint, ENCODER_TOLERANCE, motorDirections);
        } else {
            driveMecanum(thrust, strafe, 0, false);
            while (topLeft.isBusy()) { // || bottomLeft.isBusy() || topRight.isBusy() || bottomRight.isBusy()) {
                //robot moves
            }
            driveTank(0, 0);
        }
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /**
     * Used to drive in a certain direction (not F, B, L, or R -- use driveDistance() w/ dist instead of time)
     * for a certain amount of time
     *
     * @param power fraction of max power to drive with
     * @param degreeDirection unit circle direction requested to travel
     * @param milliseconds amount of time to travel for in milliseconds
     */
    public void driveMecanum(double power, double degreeDirection, double milliseconds) {
        //Calculate component values
        double thrust = Math.sin(Math.toRadians(degreeDirection)) * power;
        double strafe = Math.cos(Math.toRadians(degreeDirection)) * power;

        //Start timer
        timer.reset();
        while (timer.milliseconds() < milliseconds) {
            driveMecanum(thrust, strafe, 0, false);
        }
        driveTank(0, 0);
    }

    /**
     * Used to rotate a certain number of degrees about unit circle using encoders
     *
     * @param power the power at which to travel
     * @param degrees degree amount to turn
     *                negative value = clockwise
     *                positive value = counterclockwise
     */
    public void rotateDegrees(double power, double degrees) {
        int setPoint = (int) ((degrees / 360) * TICKS_PER_ROTATION);
        if (Math.abs(setPoint) < ENCODER_TOLERANCE) {
            return;
        }
        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setTargetPosition(-setPoint);
        bottomLeft.setTargetPosition(-setPoint);
        topRight.setTargetPosition(setPoint);
        bottomRight.setTargetPosition(setPoint);

        setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (setPoint < 0) {
            driveTank(power, -power);
        } else if (setPoint > 0) {
            driveTank(-power, power);
        }

        while (topLeft.isBusy() || bottomLeft.isBusy() || topRight.isBusy() || bottomRight.isBusy()) {
            //Yeet
        }
        driveTank(0, 0);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Uses the IMU to rotate to a certain unit circle degree direction rather than rotating a certain
     * number of degrees. Will rotate with PD control within a given tolerance to that angle.
     * NOTE: kP and kD values may change with significant changes to robot (or if this code is being
     * applied to another robot)
     *
     * @param target unit circle degree measure to which to travel
     */
    public void rotateTo(double target) {
        double kP = 0.03;
        double kD = 1.7;
        double tolerance = 2;

        double error = target - getUnitCircleAngle();
        double previousError;
        double derivative = 0;

        if (!RobotMain.godtonomous) {
            while (Math.abs(error) > tolerance &&
                    Math.abs(Math.hypot(RobotMain.gamepad1.left_stick_x, RobotMain.gamepad1.left_stick_y)) < 0.2 &&
                    Math.abs(RobotMain.gamepad1.right_stick_x) < 0.2) {
                timer.reset();
                double power = Math.abs((error * kP) + (derivative * kD));
                if (error < 0) {
                    driveTank(power, -power);
                } else {
                    driveTank(-power, power);
                }
                previousError = error;
                error = target - getUnitCircleAngle();
                derivative = (error - previousError) / timer.milliseconds() * kD;
            }
        } else {
            while (Math.abs(error) > tolerance) {
                timer.reset();
                double power = Math.abs((error * kP) + (derivative * kD));
                if (error < 0) {
                    driveTank(power, -power);
                } else {
                    driveTank(-power, power);
                }
                previousError = error;
                error = target - getUnitCircleAngle();
                derivative = (error - previousError)/timer.milliseconds() * kD;
            }
        }
        driveTank(0, 0);
    }

    /**
     * Returns the raw IMU angle without any modification.
     * Note: IMU will calibrate on init.
     * @return raw IMU angle
     */
    public double getRawAngle() {
        return RobotMain.imu.getAngularOrientation().firstAngle;
    }

    /**
     * Returns the corrected IMU angle -- the raw angle converted to a unit circle measure
     *    (0 to 359 degrees), where 0 is left and angle increases going counterclockwise
     * @return corrected unit circle angle
     */
    public double getUnitCircleAngle() {
        double rawAngle = getRawAngle();
        if (rawAngle >= -90 && rawAngle <= 180) {
            return rawAngle + 90;
        } else if (rawAngle > -180 && rawAngle < 90) {
            return 2 * (rawAngle + 180) + Math.abs(rawAngle) + 100;
        }
        return rawAngle;
    }

    /**
     * Used to set mode of all drive encoders
     *
     * @param mode mode for encoders to be set to
     */
    public void setEncoderMode(DcMotor.RunMode mode) {
        topLeft.setMode(mode);
        bottomLeft.setMode(mode);
        topRight.setMode(mode);
        bottomRight.setMode(mode);
    }

    /**
     * @param inches distance in inches
     * @return <i>inches</i> converted to encoder ticks
     */
    public int toTicks(double inches) {
        double rotations = inches / WHEEL_CIRCUMFERENCE;
        return (int) (rotations * TICKS_PER_ROTATION / GEAR_RATIO);
    }

    /**
     * @return the singleton instance of the subsystem
     */
    public static Subsystem getInstance() {
        return instance;
    }

}