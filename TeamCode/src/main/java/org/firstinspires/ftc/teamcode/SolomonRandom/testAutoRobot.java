package org.firstinspires.ftc.teamcode.SolomonRandom;

// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="testAuto", group="Robot")
@Disabled
public class testAutoRobot extends LinearOpMode {
    // declaring motor variables
    private DcMotor leftFront  = null;
    private DcMotor leftRear   = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear  = null;

    private DcMotor parallelEncoder      = null;
    private DcMotor perpendicularEncoder = null;

    // declaring sensor variables
    private IMU imu;

    // calculate the COUNTS_PER_INCH (tick to inch)
    static final double COUNTS_PER_MOTOR_REV  = 384.5;      // eg: GoBILDA 435 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION  = 1.0;        // no external gearing
    static final double WHEEL_DIAMETER_INCHES = 3.77953;    // for figuring circumference
    static final double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public void runOpMode() {
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

        // set the encoders for open loop speed control
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        parallelEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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

        //----------------------------------------------------------------------------------------------------
        // ENTER CODE BELOW:

        testDummy();

    }

    //----------------------------------------------------------------------------------------------------
    // METHODS (FUNCTIONS) (MYBLOCKS)
    //----------------------------------------------------------------------------------------------------

    public void testDummy()
    {
        // drive straight
        driveStraight(0.5, 10, 0);
        // lift arm up
        liftArm(0.25, 1000);
        // spin 90
        // extend gripper
        // drop cone
    }

    public void driveStraight(double speed, double distanceINCH, int heading)
    {
        // a lot of confusing stuff here
    }

    public void liftArm(double speed, double distanceTICK)
    {
        // a lot of confusing stuff here
    }


    public double getRawHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

}
