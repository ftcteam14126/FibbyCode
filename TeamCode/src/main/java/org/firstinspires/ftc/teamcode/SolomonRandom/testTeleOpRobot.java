package org.firstinspires.ftc.teamcode.SolomonRandom;

// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="testTeleOp", group="Robot")
@Disabled
public class testTeleOpRobot extends OpMode {
    // declaring motor variables
    private DcMotor leftFront  = null;
    private DcMotor leftRear   = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear  = null;

    private double leftFrontSpeed  = 0;
    private double leftRearSpeed   = 0;
    private double rightFrontSpeed = 0;
    private double rightRearSpeed  = 0;

    private DcMotor parallelEncoder      = null;
    private DcMotor perpendicularEncoder = null;

    // declaring sensor variables
    private IMU imu = null;

    private double robotHeading  = 0;
    private double headingOffset = 0;
    private double headingError  = 0;
    private double turnCorrection  = 0;


    // telemetry variables
    private double  targetHeading = 0;

    // CONSTANT VARIABLES
    static final double P_TURN_GAIN  = 0.02;    // larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.03;    // larger is more responsive, but also less stable



    @Override
    public void init() {
        // define and initialize motors (hardware mapping)
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        parallelEncoder      = hardwareMap.get(DcMotor.class, "parallelEncoder");
        perpendicularEncoder = hardwareMap.get(DcMotor.class, "perpendicularEncoder");

        // set the motor direction (to make it drive correctly)
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // reset the encoders and set the motors to BRAKE mode
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        // set the encoders for closed loop speed control
//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        parallelEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        perpendicularEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        // set the encoders for open loop speed control
//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        parallelEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        perpendicularEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // define and initialize servos (hardware mapping)


        // define and initialize sensors (hardware mapping)
        imu = hardwareMap.get(IMU.class, "imu");

        // define Hub orientation
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // reset the heading
        resetHeading();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {
        double y  = -gamepad1.left_stick_y;         // remember, this is reversed
        double x  = gamepad1.left_stick_x * 1.1;    // counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        turnCorrection = getSteeringCorrection(0, P_DRIVE_GAIN);

        // denominator is the largest motor power (absolute value) or 1
        // this ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // figure out the math to
        leftFrontSpeed  = ((y + x + rx) / denominator) - turnCorrection;
        leftRearSpeed   = ((y - x + rx) / denominator) - turnCorrection;
        rightFrontSpeed = ((y - x - rx) / denominator) + turnCorrection;
        rightRearSpeed  = ((y + x - rx) / denominator) + turnCorrection;

        leftFront.setPower(leftFrontSpeed);
        leftRear.setPower(leftRearSpeed);
        rightFront.setPower(rightFrontSpeed);
        rightRear.setPower(rightRearSpeed);

        sendTelemetry();
    }

    @Override
    public void stop() {}

    //----------------------------------------------------------------------------------------------------
    // METHODS (FUNCTIONS) (MYBLOCKS)
    //----------------------------------------------------------------------------------------------------
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public double getRawHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    private void sendTelemetry() {
        telemetry.addData("Luxo", "Telemetry");
        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnCorrection);

        telemetry.addLine();

        telemetry.addData("Actual Pos Front L:R",  "%7d:%7d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
        telemetry.addData("Actual Pos Rear  L:R",  "%7d:%7d", leftRear.getCurrentPosition(),  rightRear.getCurrentPosition());

        telemetry.addLine();

        telemetry.addData("Wheel Speeds Front L:R.", "%5.2f : %5.2f", leftFrontSpeed, rightFrontSpeed);
        telemetry.addData("Wheel Speeds Rear  L:R.", "%5.2f : %5.2f", leftRearSpeed,  rightRearSpeed);
        telemetry.update();

    }
}
