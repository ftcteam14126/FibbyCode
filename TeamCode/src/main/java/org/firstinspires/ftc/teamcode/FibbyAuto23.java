package org.firstinspires.ftc.teamcode;// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import android.view.View;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.SolomonRandom.MB1242Ex;

import java.util.List;

@Autonomous(name="FibbyAuto23", group="Robot")
//@Disabled
public class FibbyAuto23 extends LinearOpMode {
    //----------------------------------------------------------------------------------------------------
    // DECLARING MOTOR VARIABLES
    private DcMotorEx leftFront  = null;
    private DcMotorEx leftRear   = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightRear  = null;

    private double leftFrontPower  = 0;
    private double leftRearPower   = 0;
    private double rightFrontPower = 0;
    private double rightRearPower  = 0;

    // Sensor Variables
    private double leftFrontEncoder;
    private double leftRearEncoder;
    private double rightFrontEncoder;
    private double rightRearEncoder;
    private double leftFrontVelocity;
    private double leftRearVelocity;
    private double rightFrontVelocity;
    private double rightRearVelocity;
    private double topLiftEncoder;
    private double bottomLiftEncoder;
    private double topLiftAmps;
    private double bottomLiftAmps;
    private double parallelEncoder_reading;
    private double perpendicularEncoder_reading;
    private boolean lowerLimitSwitch;
    private double distForZero_reading;
    private double rangeSensor_reading;
    private double frontColorDist_reading;
    private double distIntake_reading;

    private DcMotorEx topLift    = null;  // these are the arm motors
    private DcMotorEx bottomLift = null;

    private double liftPower = 0;

    private DcMotor parallelEncoder      = null;    // these are the dead wheels
    private DcMotor perpendicularEncoder = null;

    //----------------------------------------------------------------------------------------------------
    // DECLARING SERVO VARIABLES
    private Servo grabber = null;
    private Servo getOutOfMyWay = null;
    
    private Servo deadWheelLift;
    private Servo conePlow;

    public Servo reach = null;

    //----------------------------------------------------------------------------------------------------
    // DECLARING SENSOR VARIABLES
    private IMU imu = null;

    private DistanceSensor distIntake;
    private DistanceSensor distForZero;
    private NormalizedColorSensor frontColorDist;
    View relativeLayout;

    DigitalChannel lowerLimit;
    DigitalChannel upperLimit;

    private MB1242Ex rangeSensor;

    //----------------------------------------------------------------------------------------------------
    // CONSTANT VARIABLES
    static final boolean SeeTelemetry = true;
    static final double P_DRIVE_GAIN = 0.03;    // larger is more responsive, but also less stable

    static final double tickToINCH = 1102;  // reading of the encoder per inch
    
    static final double plowLow  = 0.32;
    static final double plowHigh = 0.24;
    
    static final double openGrabber  = 0.03;
    static final double closeGrabber = 0.0;

    static final int plungeHeight = -160;
    static final int startHeight = 40;

    static final int poleHigh = 1340;
    static final int poleMid = 870;
    static final int poleShort = 520;

    static final int zero = 20;
    static final int tolerance = 10;

    static final int coneStack5 = 330;
    static final int coneStack4 = 265;
    static final int coneStack3 = 220;
    static final int coneStack2 = 175;
    // NOT A CONSTANT
    int grabHeight = 130;

    static final double reachOut = 0.47;
    static final double reachIn = 0;

    static final double deadwheelLift = 0.194;
    static final double deadwheelLower = 0.2357;

    static final double driveMaxVelocity = 2920; // 12 Volt battery tested at 2680, 13.73 battery was at 3120, and 13 volts was 2920

    //----------------------------------------------------------------------------------------------------
    // REUSABLE VARIABLES
    int coneImage = 0;  // for camera detection
    
    double corrHeading;
    double heading = 0;
    double diffCorrection;
    double delta;

    boolean questionAnswered = false;
    boolean rightSide = false;

    boolean onePlusThreeAuto = false;
    boolean safeParking = false;

    double desiredCourse = 0;   // telemetry use

    //----------------------------------------------------------------------------------------------------
    // LIGHT CONTROL VARIABLES
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    
    //----------------------------------------------------------------------------------------------------
    // CAMERA/VUFORIA VARIABLES

    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

    private static final String[] LABELS =
            {
                    "1 Rose",
                    "2 Snail",
                    "3 Pineapple"
            };

    private static final String VUFORIA_KEY = "Ae6MFyH/////AAABmf06rZVcN0VqtDgoLd1KtAcowILYnLgT+SXkGwocAFEpSEmZiGI51vbPAL/QfAgSIhIpPrAK3Fl+vReEgcd1kU5az1pYTI01VqIV1+3ELPbCEcnNMdw3Rs7L8tMMsyfY2nebMlSFfgn6rkfJnnoQEnr4gCGHB1K/7VVZsFg7AlG4SPJ1bORRlhkFf0xIP28Tvr80YnC06hnsL2AQgOJtGrPv7HUO04hWxe9jLMdwHhmnBu/FZfovI9A6EjrzB72joGNTzuLA5R2bBGKW6AsrkKcgw1y50EFOqanZ19gWBX7bc1OExeaNeNIMaGzbMV8jwVahVndqS4EiLc9FuudY21tw4b4jupvhSYUiSGBMtLmh\";\n";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    
    @Override
    public void runOpMode() {
        //----------------------------------------------------------------------------------------------------
        // DEFINE AND INITIALIZE MOTORS (hardware mapping)
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        parallelEncoder = hardwareMap.get(DcMotor.class, "parallelEncoder");
        perpendicularEncoder = hardwareMap.get(DcMotor.class, "perpendicularEncoder");

        topLift = hardwareMap.get(DcMotorEx.class, "Lift");
        bottomLift = hardwareMap.get(DcMotorEx.class, "Lift2");
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // SET THE MOTOR DIRECTION (to make it drive correctly)
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE); // changed 2-23-23 from forward to fix encoder issue
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);

        parallelEncoder.setDirection(DcMotor.Direction.REVERSE);
        perpendicularEncoder.setDirection(DcMotor.Direction.REVERSE);

        topLift.setDirection(DcMotorEx.Direction.REVERSE);
        bottomLift.setDirection(DcMotorEx.Direction.REVERSE);

        // RESET THE ENCODERS AND SET THE MOTORS TO BRAKE MODE
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bottomLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        topLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bottomLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // ALLOW OR NOT ALLOW ENCODERS
        topLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bottomLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // SET POWER
        topLift.setPower(0);
        bottomLift.setPower(0);

        //----------------------------------------------------------------------------------------------------
        // DEFINE AND INITIALIZE SERVOS (hardware mapping)
        grabber = hardwareMap.get(Servo.class, "grabber");
        grabber.setPosition(openGrabber);   // set grabber open during init

        conePlow = hardwareMap.get(Servo.class, "plow");
        conePlow.setPosition(plowLow);  // set grabber open during init

        deadWheelLift = hardwareMap.get(Servo.class, "Deadwheel_Lift");
        // deadWheelLift.setPosition(0.5) // it either lift the dead wheels or lowers

        reach = hardwareMap.get(Servo.class, "reach");

        //----------------------------------------------------------------------------------------------------
        // DEFINE AND INITIALIZE SENSORS (hardware mapping)
        imu = hardwareMap.get(IMU.class, "imu");

        distIntake = hardwareMap.get(DistanceSensor.class, "dist_intake2");
        distForZero = hardwareMap.get(DistanceSensor.class, "dist_for_zero");
        frontColorDist = hardwareMap.get(NormalizedColorSensor.class, "dist_intake");

        lowerLimit = hardwareMap.get(DigitalChannel.class, "l_limit");
        upperLimit = hardwareMap.get(DigitalChannel.class, "u_limit");

        lowerLimit.setMode(DigitalChannel.Mode.INPUT);
        upperLimit.setMode(DigitalChannel.Mode.INPUT);

        rangeSensor = hardwareMap.get(MB1242Ex.class, "rangeSensor");

        // DEFINE HUB ORIENTATION
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // INITIALIZE THE IMU WITH THIS MOUNTING ORIENTATION
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // RESET THE HEADING
        resetHeading();

        // LED LIGHTS
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        //----------------------------------------------------------------------------------------------------
        // DO DURING INIT
        //TESTING BLOCK -----------------------------------------

        /*
        sendTelemetry("Init");
        leftFront.setVelocity(10, AngleUnit.DEGREES);
        leftRear.setVelocity(-10, AngleUnit.DEGREES);
        rightFront.setVelocity(10, AngleUnit.DEGREES);
        rightRear.setVelocity(10, AngleUnit.DEGREES);
        int loopcount = 0;
        while (loopcount <= 100000) {
            ReadSensors();
            sendTelemetry("Init - running motors");
        }

        leftRear.setVelocity(0);
        rightRear.setVelocity(0);
        leftFront.setVelocity(0);
        rightFront.setVelocity(0);
*/
        //reset reach
        reach.setPosition(reachIn);
        // lower arm until limit switch is activated
        while (lowerLimit.getState() == true){
            topLift.setPower(-0.3);
            bottomLift.setPower(0.3);
        }
        // raise arm unit distance sensor see it
        while (distForZero.getDistance(DistanceUnit.MM) > 80) {
            topLift.setPower(0.6);
            bottomLift.setPower(-0.6);
        }

        topLift.setPower(0);
        bottomLift.setPower(0);

        topLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        topLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        bottomLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bottomLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        topLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bottomLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        liftENC(grabHeight, 1);
        sleep(750);
        topLift.setPower(0);
        bottomLift.setPower(0);

        //----------------------------------------------------------------------------------------------------
        // QUESTIONS
        telemetry.addLine("Left Side - □ | Right Side - O");
        telemetry.update();

        while (questionAnswered == false) {
            if (gamepad1.b || gamepad2.b) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
                blinkinLedDriver.setPattern(pattern);
                rightSide = true;
                questionAnswered = true;
            } else if (gamepad1.x || gamepad2.x) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
                blinkinLedDriver.setPattern(pattern);
                rightSide = false;
                questionAnswered = true;
            } else {
                questionAnswered = false;
            }

            if (questionAnswered == true) {
                telemetry.clear();
                telemetry.update();
            }
        }

        telemetry.addLine("conedrop - dpadup | safeParking - dpaddown");
        telemetry.update();

        questionAnswered = false;
        while (questionAnswered == false){
            if (gamepad1.dpad_up || gamepad2.dpad_up){
                onePlusThreeAuto = true;
                questionAnswered = true;
            }
            else if (gamepad1.dpad_down || gamepad2.dpad_down){
                safeParking = true;
                questionAnswered = true;
            }
            else
            {
                questionAnswered = false;
            }

            if (questionAnswered == true) {
                telemetry.clear();
                telemetry.update();
            }

        }
        //----------------------------------------------------------------------------------------------------
        // CAMERA/VUFORIA THINGS

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.0, 16.0/9.0);
        }

        while (!isStarted()) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                        // this allows un to store the image that camera sees and use it in our cone
                        if (recognition.getLabel() == "1 Rose") {
                            coneImage = 1;
                            if (rightSide == true) {
                                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                                blinkinLedDriver.setPattern(pattern);
                            }
                            else {
                                pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_RED;
                                blinkinLedDriver.setPattern(pattern);
                            }
                        }
                        else if (recognition.getLabel() == "2 Snail") {
                            coneImage = 2;
                            pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE;
                            blinkinLedDriver.setPattern(pattern);
                        }
                        else if (recognition.getLabel() == "3 Pineapple") {
                            coneImage = 3;
                            pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;
                            blinkinLedDriver.setPattern(pattern);
                        }
                        else {
                            coneImage = 0;
                        }
                    }
                    telemetry.update();
                }
            }
            
            // someone need to explain this to me
            if (gamepad2.left_bumper|| gamepad1.left_bumper) {
                conePlow.setPosition(plowLow);
                liftENC(plungeHeight, -0.5);
                sleep(500);
            }
            if (topLift.getCurrentPosition() <= -100 && grabber.getPosition() != closeGrabber){
                topLift.setPower(0);
                bottomLift.setPower(0);
                grabber.setPosition(closeGrabber);
                sleep(500);
                liftENC(startHeight + 20, 1);
                sleep(1000);
                topLift.setPower(0);
                bottomLift.setPower(0);
                conePlow.setPosition(plowHigh);
            }
        }

        //----------------------------------------------------------------------------------------------------
        // ENTER CODE BELOW:
        if (onePlusThreeAuto){
            resetRuntime();
            coneDrop();
        }
        else if (safeParking) {
            safeParking();
        }
        else {
            // do nothing
        }
    }


    //----------------------------------------------------------------------------------------------------
    // OUR AUTONOMOUS RUNS
    //----------------------------------------------------------------------------------------------------

    public void coneDrop()
    {
        if (rightSide) {
            conePlow.setPosition(plowHigh);

            liftENC(poleHigh, 1);

            GyroDriveENC(22, 0.7, 0, true, true, true, false);
            GyroDriveENC(29, 0.35, 0, false, true, true, false);                     // 32 delta
            GyroDriveENC(54, 0.2, 0, false, true, true, true);

            topLift.setPower(0);
            bottomLift.setPower(0);

            GyroDriveENC(50, .25, 0, false, false, true, true);
            GyroSpin(0.5, -42);
            reach.setPosition(reachOut);

            // move into pole a little and drop cone
            GyroDriveENC(3.5, 0.5, -44, true, true, true, true);
            grabber.setPosition(openGrabber);
            sleep(175);
            reach.setPosition(reachIn);

            // back up a tad
            GyroDriveENC(-2, 0.4, -44, true, false, true, true);

            //turn to stack
            GyroSpin(0.55, 88);

            // lift arm and drive to grab cone from stack
            liftENC(coneStack5, -0.6);
            GyroDriveENC(10, 0.7, 91.5, true, true, true, true);
            GyroDriveDistStack(0.25, 90, true, 5);

//        GyroDriveENC(4, 0.5, 90, true, false);
            double RangedDistance = rangeSensor.getDistance(DistanceUnit.INCH); // how far away are the poles?
            if (RangedDistance <= 28 || RangedDistance >= 32)
                RangedDistance = -29;
            else
                RangedDistance = -RangedDistance;
            GyroDriveENC(RangedDistance + 12, 0.6, 90, true, false, true, true);
            liftENC(poleHigh, 1);

            reach.setPosition(reachOut);
            GyroSpin(0.5, -20);

            topLift.setPower(0);
            bottomLift.setPower(0);

            //GyroDriveENC(1, 0.3, -35, true, true, true, true);
            grabber.setPosition(openGrabber);
            sleep(250);
            reach.setPosition(reachIn);


//########################################### CONE #2 ###########################################
            // back up a tad
            //GyroDriveENC(-2, 0.4, -35, true, false, true, true);

            //turn to stack
            GyroSpin(0.55, 88);

            // lift arm and drive to grab cone from stack
            liftENC(coneStack5, -0.6);
            GyroDriveENC(10, 0.7, 92, true, true, true, true);
            GyroDriveDistStack(0.25, 90, true, 4);

            // go to encoder 0
//        GyroDriveENC(4, 0.5, 90, true, false);
            RangedDistance = rangeSensor.getDistance(DistanceUnit.INCH); // how far away are the poles?
            if (RangedDistance <= 28 || RangedDistance >= 32)
                RangedDistance = -29;
            else
                RangedDistance = -RangedDistance;
            GyroDriveENC(RangedDistance + 12, 0.6, 90, true, false, true, true);
            liftENC(poleHigh, 1);

            reach.setPosition(reachOut);
            GyroSpin(0.5, -20);

            topLift.setPower(0);
            bottomLift.setPower(0);

            //GyroDriveENC(1, 0.3, -35, true, true, true, true);
            grabber.setPosition(openGrabber);
            sleep(175);
            reach.setPosition(reachIn);

//########################################### CONE #3 ###########################################
            // back up a tad
            //GyroDriveENC(-2, 0.4, -35, true, false, true, true);

            //turn to stack
            GyroSpin(0.55, 88);

            // lift arm and drive to grab cone from stack
            liftENC(coneStack5, -0.6);
            GyroDriveENC(10, 0.7, 92, true, true, true, true);
            GyroDriveDistStack(0.25, 90, true, 3);

            // go to encoder 0
//        GyroDriveENC(4, 0.5, 90, true, false);
            RangedDistance = rangeSensor.getDistance(DistanceUnit.INCH); // how far away are the poles?
            if (RangedDistance <= 28 || RangedDistance >= 32)
                RangedDistance = -29;
            else
                RangedDistance = -RangedDistance;
            GyroDriveENC(RangedDistance + 12, 0.6, 90, true, false, true, true);
            liftENC(poleHigh, 1);

            reach.setPosition(reachOut);
            GyroSpin(0.5, -20);

            topLift.setPower(0);
            bottomLift.setPower(0);

            //GyroDriveENC(1, 0.3, -35, true, true, true, true);
            grabber.setPosition(openGrabber); //let go of Cone
            sleep(175);
            //GyroDriveENC(-2.5, 0.3, -35, true, false);
            reach.setPosition(reachIn);
            //GyroDriveENC(-1.75, 0.4, -35, true, false, true, true);

            //################################## PARK!! ###########################
            if (coneImage == 1) {             //Rose
                liftENC(coneStack5, 0.6);
                GyroSpin(0.5, -90);
                GyroDriveENC(12, 0.5, -90, true, true, true, true);
            } else if (coneImage == 2) {         //Snail
                liftENC(coneStack5, 0.6);
                GyroSpin(0.55, -90);
                GyroDriveENC(-3, 0.5, -90, true, false, true, true);
            } else {                           //Pineapple
                //turn to stack
                liftENC(coneStack5, 0.6);
                GyroSpin(0.55, 88);
                // lift arm and drive to grab cone from stack
                //liftENC(coneStack5, -0.6);
                GyroDriveENC(15, 0.6, 90, true, true, true, true);
                //GyroDriveDistStack(0.2, 90, true, 2);
            }
        }

        //############################################################# left side
        else {
            conePlow.setPosition(plowHigh);

            liftENC(poleHigh, 1);

            GyroDriveENC(22, 0.7, 0, true, true, true, false);
            GyroDriveENC(29, 0.35, 0, false, true, true, false);                     // 32 delta
            GyroDriveENC(54, 0.2, 0, false, true, true, true);

            topLift.setPower(0);
            bottomLift.setPower(0);

            GyroDriveENC(50, .25, 0, false, false, true, true);
            GyroSpin(0.6, 38);
            reach.setPosition(reachOut);

            // move into pole a little and drop cone
            GyroDriveENC(2.7, 0.5, 40, true, true, true, true);
            grabber.setPosition(openGrabber);
            sleep(150);
            reach.setPosition(reachIn);

            // back up a tad
            GyroDriveENC(-3, 0.4, 44, true, false, true, true);

            //turn to stack
            GyroSpin(0.6, -89);

            // lift arm and drive to grab cone from stack
            liftENC(coneStack5, -0.6);
            GyroDriveENC(13, 0.7, -91, true, true, true, true);
            GyroDriveDistStack(0.25, -90, true, 5);

            double RangedDistance = rangeSensor.getDistance(DistanceUnit.INCH); // how far away are the poles?
            if (RangedDistance <= 28 || RangedDistance >= 32)
                RangedDistance = -29;
            else
                RangedDistance = -RangedDistance;
            GyroDriveENC(RangedDistance + 13, 0.6, -92, true, false, true, true);
            liftENC(poleHigh, 1);

            reach.setPosition(reachOut);
            GyroSpin(0.6, 24.5);

            topLift.setPower(0);
            bottomLift.setPower(0);

            grabber.setPosition(openGrabber);
            sleep(175);
            reach.setPosition(reachIn);
            ////////////////////////////////// Cone 2 ////////////////////////////////////////
            if (getRuntime() <= 23) {

            GyroSpin(0.55, -90);

            // lift arm and drive to grab cone from stack
            liftENC(coneStack5, -0.6);
            GyroDriveENC(13, 0.7, -91, true, true, true, true);
            GyroDriveDistStack(0.25, -90, true, 4);

            RangedDistance = rangeSensor.getDistance(DistanceUnit.INCH); // how far away are the poles?
            if (RangedDistance <= 28 || RangedDistance >= 32)
                RangedDistance = -29;
            else
                RangedDistance = -RangedDistance;
            GyroDriveENC(RangedDistance + 11.5, 0.6, -91, true, false, true, true);
            liftENC(poleHigh, 1);

            reach.setPosition(reachOut);
            GyroSpin(0.6, 18);

            topLift.setPower(0);
            bottomLift.setPower(0);

            grabber.setPosition(openGrabber);
            sleep(175);
            reach.setPosition(reachIn);
        }
////////////////////////////////// Cone 3 ////////////////////////////////////////
            if (getRuntime() <= 23) {
                GyroSpin(0.6, -90);

                // lift arm and drive to grab cone from stack
                liftENC(coneStack5, -0.6);
                GyroDriveENC(10, 0.7, -91, true, true, true, true);
                GyroDriveDistStack(0.23, -90, true, 3);

                RangedDistance = rangeSensor.getDistance(DistanceUnit.INCH); // how far away are the poles?
                if (RangedDistance <= 28 || RangedDistance >= 32)
                    RangedDistance = -29;
                else
                    RangedDistance = -RangedDistance;
                GyroDriveENC(RangedDistance + 11.5, 0.6, -90.5, true, false, true, true);
                liftENC(poleHigh, 1);

                reach.setPosition(reachOut);
                GyroSpin(0.6, 18);

                topLift.setPower(0);
                bottomLift.setPower(0);

                grabber.setPosition(openGrabber);
                sleep(175);
                reach.setPosition(reachIn);
            }
            if ((getRuntime() <= 23) && (coneImage == 2)){
                GyroSpin(0.6, -90);

                // lift arm and drive to grab cone from stack
                liftENC(coneStack5, -0.6);
                GyroDriveENC(10, 0.7, -91, true, true, true, true);
                GyroDriveDistStack(0.23, -90, true, 2);

                RangedDistance = rangeSensor.getDistance(DistanceUnit.INCH); // how far away are the poles?
                if (RangedDistance <= 28 || RangedDistance >= 32)
                    RangedDistance = -29;
                else
                    RangedDistance = -RangedDistance;
                GyroDriveENC(RangedDistance + 11.5, 0.6, -90.5, true, false, true, true);
                liftENC(poleHigh, 1);

                reach.setPosition(reachOut);
                GyroSpin(0.6, 18);

                topLift.setPower(0);
                bottomLift.setPower(0);

                grabber.setPosition(openGrabber);
                sleep(175);
                reach.setPosition(reachIn);
            }

            ////////////////////////////////////////////////Park////////////////////////////////////
            if (coneImage == 1) {             //Rose
                //turn to stack
                liftENC(coneStack5, 0.6);
                GyroSpin(0.55, -88);
                GyroDriveENC(15, 0.6, -90, true, true, true, true);
            } else if (coneImage == 2) {         //Snail
                liftENC(coneStack5, 0.6);
                GyroSpin(0.7, 87);
                GyroDriveENC(-2, 0.9, 90, true, false, true, true);
            } else {                        //Pineapple
                liftENC(coneStack5, 0.6);
                GyroSpin(0.5, 90);
                GyroDriveENC(10.2, 0.6, 90, true, true, true, true);

            }
        }
    }

    public void safeParking ()
    {
        GyroDriveENC(20, 0.5, 0, true, true, true, true);
        if (coneImage == 1){
            GyroStrafeENC(20, 0.5, "left", 0);
        }
        else if (coneImage == 3){
            GyroStrafeENC(20, 0.5, "right", 0);
        }
    }

    //----------------------------------------------------------------------------------------------------
    // FUNCTIONS (MYBLOCKS)
    //----------------------------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------------------------
    // GyroDriveENC
    public void GyroDriveENC(double distance, double power, double course, boolean reset, boolean forward, boolean rampdown, boolean stopmotors) {
        // for telemetry
        desiredCourse = course;

        // turn off encoders for drive wheels
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // if reset is true, then reset the deal wheel encoders
        if (reset == true) {
            parallelEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            perpendicularEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        // set brake for drive wheels
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // calcuate the motor power for direction
        power = Math.abs(power);           //this ensures that if we are moving forward it is positive
        if (!forward) {power = -power;}    // and if we are going backwards it is negative

        double originPower = power;

        ReadSensors();
        while ((parallelEncoder_reading < distance && forward && opModeIsActive())
                || // OR
                (parallelEncoder_reading > distance && !forward && opModeIsActive())) {
            //ReadSensors();


            parallelEncoder_reading = parallelEncoder.getCurrentPosition()/tickToINCH;
            //topLiftAmps = topLift.getCurrent(CurrentUnit.AMPS);
            //bottomLiftAmps = bottomLift.getCurrent(CurrentUnit.AMPS);


            //sendTelemetry("GyroDriveENC");
            getRawHeading();
           // armMonitor();
            corrHeading = course - heading;
            diffCorrection = Math.abs(corrHeading * P_DRIVE_GAIN);

            if (corrHeading < 0) { //robot is drifting to the right, so we need to correct to the left
                leftFrontPower = (power - diffCorrection);
                leftRearPower = (power - diffCorrection);
                rightFrontPower = (power + diffCorrection);
                rightRearPower = (power + diffCorrection);
            }
            else if (corrHeading > 0){  //robot is drifting to the left, so we need to correct to the right
                leftFrontPower = (power + diffCorrection);
                leftRearPower = (power + diffCorrection);
                rightFrontPower = (power - diffCorrection);
                rightRearPower = (power - diffCorrection);
            }
            else
            {
                leftFrontPower = (power);
                leftRearPower = (-power);
                rightFrontPower = (power);
                rightRearPower = (power);
            }

            if (stopmotors) {
                // ramping down
                delta = Math.abs(Math.abs(distance) - Math.abs(parallelEncoder_reading));
                int deltaThreshold = 6;
                if ((delta <= deltaThreshold) && (forward) && (rampdown)) {
                    //if (power > 0.2) {
                    power = (delta / deltaThreshold) * originPower;
                    // }
                }

                if ((delta <= deltaThreshold) && (!forward) && (rampdown)) {
                    //if (power < -0.2){
                    power = (delta / deltaThreshold) * originPower;
                    //}
                }
            }

            leftFront.setVelocity(leftFrontPower * driveMaxVelocity);
            leftRear.setVelocity(-leftRearPower * driveMaxVelocity);
            rightFront.setVelocity(rightFrontPower * driveMaxVelocity);
            rightRear.setVelocity(rightRearPower * driveMaxVelocity);
            /*
            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower); */
        }
        if (stopmotors) {


            leftFront.setVelocity(0);
            rightFront.setVelocity(0);
            leftRear.setVelocity(0);
            rightRear.setVelocity(0);
        }
        /*
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
*/
    }


    //----------------------------------------------------------------------------------------------------
    // GyroStrafeENCStack
    public void GyroDriveDistStack(double power, double course, boolean reset, int stack) {
        // for telemetry

        desiredCourse = course;

        // turn off encoders for drive wheels
      /*  leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
*/
        // if reset is true, then reset the deal wheel encoders
        if (reset == true) {
            parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // set brake for drive wheels
     /*   leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

      */

        // set conePlow to low so it will not break
        conePlow.setPosition(plowLow);
        ReadSensors();
        while ((stack == 1 &&  (distIntake_reading <= 50 || distIntake_reading >= 67))  // single cone
                ||  // OR
                (stack >= 2 && (distIntake_reading <= 30 || distIntake_reading >= 50))// stack of Cones
                && 
                (opModeIsActive())) {
                ReadSensors();
                getRawHeading();
            sendTelemetry("GyroDriveStack");
                corrHeading = course - heading;
                diffCorrection = Math.abs(corrHeading * P_DRIVE_GAIN);
    
                if (corrHeading < 0) { //robot is drifting to the right, so we need to correct to the left
                    leftFrontPower = (power - diffCorrection);
                    leftRearPower = (power - diffCorrection);
                    rightFrontPower = (power + diffCorrection);
                    rightRearPower = (power + diffCorrection);
                } else {  //robot is drifting to the left, so we need to correct to the right
                    leftFrontPower = (power + diffCorrection);
                    leftRearPower = (power + diffCorrection);
                    rightFrontPower = (power - diffCorrection);
                    rightRearPower = (power - diffCorrection);
                }
    
                leftFront.setPower(leftFrontPower);
                leftRear.setPower(-leftRearPower);
                rightFront.setPower(rightFrontPower);
                rightRear.setPower(rightRearPower);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        if (stack == 5) {liftENC(coneStack5 - 160, -0.9);}
        else if (stack == 4) {liftENC(coneStack4 - 160, -0.8);}
        else if (stack == 3) {liftENC(coneStack3 -160, -0.8);}
        else if (stack == 2) {liftENC(coneStack2 - 160, -0.8);}
        else if (stack == 1) {liftENC(plungeHeight, 0.6);}

        sleep(400);

        grabber.setPosition(closeGrabber);
        sleep(300);

        liftENC(415, 0.75);
        conePlow.setPosition(plowHigh);
        sleep(250);
    }
    
    
    //----------------------------------------------------------------------------------------------------
    // GyroStrafeENC
    public void GyroStrafeENC(double distance, double power, String direction, double course) {
        // for telemetry
        desiredCourse = course;
        ReadSensors();
        distance = Math.abs(distance);
        //direction = toString().toLowerCase(direction);
        
        // turn off encoders for drive wheels
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // reset the deal wheel encoders
        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set brake for drive wheels
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (Math.abs(perpendicularEncoder_reading) <= distance && (opModeIsActive())) {
            getRawHeading();
            ReadSensors();
            corrHeading = course - heading;
            diffCorrection = Math.abs(corrHeading * P_DRIVE_GAIN);

            if ((corrHeading < 0 && direction == "left") || (corrHeading > 0 && direction == "right")) {
                leftFrontPower = (power + diffCorrection);
                leftRearPower = (power - diffCorrection);
                rightFrontPower = (power + diffCorrection);
                rightRearPower = (power - diffCorrection);
            }
            else {
                leftFrontPower = (power - diffCorrection);
                leftRearPower = (power + diffCorrection);
                rightFrontPower = (power - diffCorrection);
                rightRearPower = (power + diffCorrection);
            }

            if(direction == "left") {
                leftFront.setPower(-leftFrontPower);
                leftRear.setPower(-leftRearPower);
                rightFront.setPower(rightFrontPower);
                rightRear.setPower(-rightRearPower);
            }
            else if(direction == "right") {
                leftFront.setPower(leftFrontPower);
                leftRear.setPower(leftRearPower);
                rightFront.setPower(-rightFrontPower);
                rightRear.setPower(rightRearPower);
            }
            else {
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
            }
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
    

    public void ReadSensors ()
    {
    rightFrontEncoder = rightFront.getCurrentPosition();
    rightRearEncoder = rightRear.getCurrentPosition();
    leftFrontEncoder = leftFront.getCurrentPosition();
    leftRearEncoder = leftRear.getCurrentPosition();
    topLiftEncoder = topLift.getCurrentPosition();
    bottomLiftEncoder = bottomLift.getCurrentPosition();
    distForZero_reading = distForZero.getDistance(DistanceUnit.MM);
    distIntake_reading = distIntake.getDistance(DistanceUnit.MM);
    frontColorDist_reading = ((DistanceSensor) frontColorDist).getDistance(DistanceUnit.MM);
    lowerLimitSwitch = lowerLimit.getState();
    perpendicularEncoder_reading = perpendicularEncoder.getCurrentPosition()/tickToINCH;
    parallelEncoder_reading = parallelEncoder.getCurrentPosition()/tickToINCH;
    topLiftAmps = topLift.getCurrent(CurrentUnit.AMPS);
    bottomLiftAmps = bottomLift.getCurrent(CurrentUnit.AMPS);
    leftFrontVelocity = leftFront.getVelocity();
    leftRearVelocity = leftRear.getVelocity();
    rightFrontVelocity = rightFront.getVelocity();
    rightRearVelocity = rightRear.getVelocity();


    }
    //----------------------------------------------------------------------------------------------------
    // GyroSpin
    public void GyroSpin(double power, double course) {
        // for telemetry
        desiredCourse = course;

        // turn off encoders for drive wheels
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // set break to drive wheels
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        getRawHeading();

        corrHeading = course - heading;
        diffCorrection = Math.abs(corrHeading * P_DRIVE_GAIN);
        
        if (heading >= course) {
            while (heading >= course) {
                getRawHeading();
                leftFrontPower = (-power);
                leftRearPower = (-power);
                rightFrontPower = (power);
                rightRearPower = (power);

                leftFront.setPower(leftFrontPower);
                leftRear.setPower(-leftRearPower);
                rightFront.setPower(rightFrontPower);
                rightRear.setPower(rightRearPower);
            }
        }
        else if (heading <= course) {
            while (heading <= course) {
                getRawHeading();
                leftFrontPower = (power);
                leftRearPower = (power);
                rightFrontPower = (-power);
                rightRearPower = (-power);

                leftFront.setPower(leftFrontPower);
                leftRear.setPower(-leftRearPower);
                rightFront.setPower(rightFrontPower);
                rightRear.setPower(rightRearPower);


            }
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
    
    //----------------------------------------------------------------------------------------------------
    public void armMonitor(){
        if (topLiftAmps > 7 || bottomLiftAmps > 7){
            topLift.setPower(0);
            bottomLift.setPower(0);
        }
    }


    // liftENC
    public void liftENC(int distance, double power) {
        if (distance > 1400) {distance = 1400;}
        
        topLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bottomLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        topLift.setTargetPosition(distance);
        bottomLift.setTargetPosition(-distance);
        
        topLift.setPower(power);
        bottomLift.setPower(-power);
        
        topLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bottomLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


    }

    //----------------------------------------------------------------------------------------------------
    // CAMERA/VUFORIA FUNCTIONS
    private void initVuforia()
    {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    //----------------------------------------------------------------------------------------------------
    // GYRO FUNCTIONS
    public void getRawHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        heading = -orientation.getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    private void sendTelemetry(String codethatsrunning) {
        if (SeeTelemetry) {
            telemetry.addData("Luxo Telemetry - Function:", codethatsrunning);
            telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", desiredCourse, heading);
//        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", corrHeading, diffCorrection);
//
//        telemetry.addData("Actual Pos Front L:R",  "%7d:%7d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
//        telemetry.addData("Actual Pos Rear  L:R",  "%7d:%7d", leftRear.getCurrentPosition(),  rightRear.getCurrentPosition());

//
            telemetry.addData("Wheel Speeds Front L:R.", "%5.2f : %5.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Wheel Speeds Rear  L:R.", "%5.2f : %5.2f", leftRearPower, rightRearPower);
//
            telemetry.addData("Actual Pos Para:Perp", "%7.0f:%7.0f", parallelEncoder_reading, perpendicularEncoder_reading);


            telemetry.addData("rangeSensor", rangeSensor_reading);
            telemetry.addData("Delta:", delta);
            telemetry.addData("Lift ENC T/B, Amps T/B", "%7f : %7f : %5.2f : %5.2f", topLiftEncoder, bottomLiftEncoder, topLiftAmps, bottomLiftAmps);
            telemetry.addData("LiftPower", liftPower);
            telemetry.addData("Velocity - LF, RF, LR, RR:", "%4f : %4f : %4f : %4f", leftFrontVelocity, rightFrontVelocity, leftRearVelocity, rightRearVelocity);
            telemetry.addData("Runtime", getRuntime());

        }
        else
            telemetry.addData("No Telemetry", "For You!");
        telemetry.update();

    }
}
