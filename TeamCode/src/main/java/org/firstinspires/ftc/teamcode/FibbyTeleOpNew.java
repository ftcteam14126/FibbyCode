package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import android.view.View;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.SolomonRandom.MB1242Ex;

@TeleOp(name="FibbyTeleOpNew", group="Robot")
//@Disabled
public class FibbyTeleOpNew extends OpMode {
    //----------------------------------------------------------------------------------------------------
    // DECLARING MOTOR VARIABLES
    private DcMotor leftFront  = null;
    private DcMotor leftRear   = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear  = null;

    private double leftFrontPower  = 0;
    private double leftRearPower   = 0;
    private double rightFrontPower = 0;
    private double rightRearPower  = 0;

    private DcMotor topLift    = null;  // these are the arm motors
    private DcMotor bottomLift = null;

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

    DigitalChannel ledGreen;
    DigitalChannel ledRed;

    private MB1242Ex rangeSensor;

    //----------------------------------------------------------------------------------------------------
    // CONSTANT VARIABLES
    static final double P_DRIVE_GAIN = 0.03;    // larger is more responsive, but also less stable

    static final double tickToINCH = 1068;  // reading of the encoder per inch
    
    static final double plowLow  = 0.32;
    static final double plowHigh = 0.24;
    
    static final double openGrabber  = 0.05;
    static final double closeGrabber = 0.0;

    static final int plungeHeight = -160;
    static final int startHeight = 40;

    static final int poleHigh = 1300;
    static final int poleMid = 850;
    static final int poleShort = 485;

    static final int zero = 20;
    static final int tolerance = 10;

    double raisePower = 0.9;
    double lowerPower = 0.6;

    static final int coneStack5 = 240;
    static final int coneStack4 = 195;
    static final int coneStack3 = 150;
    static final int coneStack2 = 95;
    // NOT CONSTANT
    int grabHeight = 150;

    static final double reachIn = 0.47;
    static final double reachOut = 0;

    //----------------------------------------------------------------------------------------------------
    // REUSABLE VARIABLES
    private ElapsedTime runtime = new ElapsedTime();
    double timeLeft = 120;

    double corrHeading;
    double heading = 0;
    double diffCorrection;

    double desiredCourse = 0;

    int step = 0;
    double cycles = 0;
    char liftButton = 'n';

    boolean yoinkMode = false;
    boolean seeCone = false;
    double coneNumber = 1;

    boolean armDestSuccess = false;

    boolean displaytelementry = false;

    //----------------------------------------------------------------------------------------------------
    // LIGHT CONTROL VARIABLES
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    @Override
    public void init() {
        //----------------------------------------------------------------------------------------------------
        // DEFINE AND INITIALIZE MOTORS (hardware mapping)
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        parallelEncoder      = hardwareMap.get(DcMotor.class, "parallelEncoder");
        perpendicularEncoder = hardwareMap.get(DcMotor.class, "perpendicularEncoder");
        
        topLift = hardwareMap.get(DcMotor.class,"Lift");
        bottomLift = hardwareMap.get(DcMotor.class,"Lift2");
        
        // SET THE MOTOR DIRECTION (to make it drive correctly)
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        parallelEncoder.setDirection(DcMotor.Direction.REVERSE);
        perpendicularEncoder.setDirection(DcMotor.Direction.REVERSE);

        topLift.setDirection(DcMotor.Direction.REVERSE);
        bottomLift.setDirection(DcMotor.Direction.REVERSE);

        // RESET THE ENCODERS AND SET THE MOTORS TO BRAKE MODE
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
       /* leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        topLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ALLOW OR NOT ALLOW ENCODERS
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // SET POWER
        topLift.setPower(0);
        bottomLift.setPower(0);

        //----------------------------------------------------------------------------------------------------
        // DEFINE AND INITIALIZE SERVOS (hardware mapping)
        grabber = hardwareMap.get(Servo .class,"grabber");
        grabber.setPosition(openGrabber);   // set grabber open during init

        conePlow = hardwareMap.get(Servo.class, "plow");
        conePlow.setPosition(plowHigh);  // set grabber open during init

        getOutOfMyWay = hardwareMap.get(Servo.class,"getoutofmyway");
        getOutOfMyWay.setPosition(0);

        deadWheelLift = hardwareMap.get(Servo.class,"Deadwheel_Lift");
        // deadWheelLift.setPosition(0.5) // it either lift the dead wheels or lowers

        reach = hardwareMap.get(Servo.class, "reach");

        //----------------------------------------------------------------------------------------------------
        // DEFINE AND INITIALIZE SENSORS (hardware mapping)
        imu = hardwareMap.get(IMU.class, "imu");

        distIntake     = hardwareMap.get(DistanceSensor.class, "dist_intake2");
        distForZero    = hardwareMap.get(DistanceSensor.class, "dist_for_zero");
        frontColorDist = hardwareMap.get(NormalizedColorSensor.class, "dist_intake");
        
        lowerLimit = hardwareMap.get(DigitalChannel.class, "l_limit");
        upperLimit = hardwareMap.get(DigitalChannel.class, "u_limit");

        lowerLimit.setMode(DigitalChannel.Mode.INPUT);
        upperLimit.setMode(DigitalChannel.Mode.INPUT);

        rangeSensor = hardwareMap.get(MB1242Ex.class, "rangeSensor");

        // DEFINE HUB ORIENTATION
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // INITIALIZE THE IMU WITH THIS MOUNTING ORIENTATION
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // RESET THE HEADING
        resetHeading();

        // LED LIGHTS
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        ledGreen = hardwareMap.get(DigitalChannel.class, "led1_green");
        ledRed = hardwareMap.get(DigitalChannel.class, "led1_red");

        ledGreen.setMode(DigitalChannel.Mode.OUTPUT);
        ledRed.setMode(DigitalChannel.Mode.OUTPUT);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        pattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;
        blinkinLedDriver.setPattern(pattern);

        //----------------------------------------------------------------------------------------------------
        // DO ACTION IN INIT
        /*
        while (lowerLimit.getState() == true) {
            topLift.setPower(-0.25);
            bottomLift.setPower(0.25);
        }
        while (distForZero.getDistance(DistanceUnit.MM) > 80) {
            topLift.setPower(0.5);
            bottomLift.setPower(-0.5);
        }
        */
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
        deadWheelLift.setPosition(0.215);
    }

    @Override
    public void loop() {
        timeLeft = 120 - runtime.seconds();
        double leftFrontPower;
        double leftRearPower;
        double rightFrontPower;
        double rightRearPower;

        if (displaytelementry) {
            getRawHeading();
            sendTelemetry();
        }

        corrHeading = 0 - heading;
        diffCorrection = Math.abs(corrHeading * P_DRIVE_GAIN);

        // CAN WE CHANGE THIS PLEASE!!!!!!!!!!!!!!!!!!!
        double finesse = 0.35;
        if (gamepad1.right_stick_y != 0) {
            if (gamepad1.left_stick_x < 0) {
                leftFrontPower = -gamepad1.right_stick_y + (gamepad1.left_stick_x * finesse);
                rightFrontPower = -gamepad1.right_stick_y;
                leftRearPower = -gamepad1.right_stick_y + (gamepad1.left_stick_x * finesse);
                rightRearPower = -gamepad1.right_stick_y;
            }
            else if (gamepad1.left_stick_x > 0) {
                leftFrontPower = -gamepad1.right_stick_y;
                rightFrontPower = -gamepad1.right_stick_y - gamepad1.left_stick_x * finesse;
                leftRearPower = -gamepad1.right_stick_y;
                rightRearPower = -gamepad1.right_stick_y - gamepad1.left_stick_x * finesse;
            }
            else {
                leftFrontPower = -gamepad1.right_stick_y;
                rightFrontPower = -gamepad1.right_stick_y;
                leftRearPower = -gamepad1.right_stick_y;
                rightRearPower = -gamepad1.right_stick_y;
            }
        }

        else if (gamepad1.left_stick_x != 0) {
            leftFrontPower = gamepad1.left_stick_x*0.75;
            rightFrontPower = -gamepad1.left_stick_x*0.75;
            leftRearPower = gamepad1.left_stick_x*0.75;
            rightRearPower = -gamepad1.left_stick_x*0.75;
        } else if (gamepad1.left_trigger != 0) { //if left trigger is being pressed it spinning left
            leftFrontPower = -gamepad1.left_trigger;
            rightFrontPower = gamepad1.left_trigger;
            leftRearPower = gamepad1.left_trigger;
            rightRearPower = -gamepad1.left_trigger;
        } else if (gamepad1.right_trigger != 0) {
            leftFrontPower = gamepad1.right_trigger;
            rightFrontPower = -gamepad1.right_trigger;
            leftRearPower = -gamepad1.right_trigger;
            rightRearPower = gamepad1.right_trigger;
        } else {
            leftFrontPower = 0;
            rightFrontPower = 0;
            leftRearPower = 0;
            rightRearPower = 0;
        }

        if (gamepad1.dpad_up){
            while (lowerLimit.getState() == true){
                topLift.setPower(-0.25);
                bottomLift.setPower(0.25);
            }
            while (distForZero.getDistance(DistanceUnit.MM) > 80) {
                topLift.setPower(0.4);
                bottomLift.setPower(-0.4);
            }
            topLift.setPower(0);
            bottomLift.setPower(0);

            topLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            topLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            bottomLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bottomLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            topLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bottomLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        //Test Code
        if (gamepad1.dpad_left){
            grabber.setPosition(closeGrabber);
        }
        if (gamepad1.dpad_right){
            grabber.setPosition(openGrabber);
        }

        // Limit switch - false == pressed, true == not pressed
        // This if statement is broken into two chunks, if the dri ver is trying to lower the lift AND the lower limit is NOT pressed,
        // OOOOORRRRRR if the driver is trying to raise the lift and the upper limit switch is not pressed then run the lift motor, otherwise don't.
        if (gamepad2.a) {
            liftButton = 'a';
            yoinkMode=false;
        } else if (gamepad2.b) {
            liftButton = 'b';
            yoinkMode=false;
        } else if (gamepad2.x) {
            liftButton = 'x';
            yoinkMode=false;
        } else if (gamepad2.y) {
            liftButton = 'y';
            yoinkMode=false;
        }

        // OLD CODE?
        /*
        else if (gamepad2.dpad_left) {
            liftButton = 'l';
            yoinkMode=false;
        }
        */

        // safety to make sure we don't break ourselves
        if ((topLift.getCurrentPosition() >= poleHigh + 50) || (lowerLimit.getState() == true)) {liftPower = 0;}

        // OLD CODE?
        /*
        if ((gamepad2.right_stick_y > 0 && lowerLimit.getState() == true) || (gamepad2.right_stick_y < 0 && topLift.getCurrentPosition() <= poleHigh+50))
        {
            if(gamepad2.right_stick_y < 0) {liftPower = gamepad2.right_stick_y;}
            else {liftPower = gamepad2.right_stick_y * .75;}
        }
        else {liftButton = 'n';}
        */

        if (gamepad2.right_stick_y != 0) {
            liftButton = 'n';
        }
        else if (liftButton == 'a') { // arm to drive height//
            ArmHeight(startHeight, 0.75);
            if (armDestSuccess) {
                liftButton = 'n';
                liftPower = 0.0;
            }
        }
        else if (liftButton == 'b') {   // arm to low junction
            {
                ArmHeight(poleShort, 0.85);
                if (armDestSuccess) {liftButton = 'n';}
            }
        }
        else if (liftButton == 'x') {   // arm to medium junction
            ArmHeight(poleMid, 1);
            if (armDestSuccess) {liftButton = 'n';}
        }
        else if (liftButton == 'y') {   // arm to high junction
            ArmHeight(poleHigh, 1);
            if (armDestSuccess) {liftButton = 'n';}
        }
        else if (liftButton == 'l') {   // arm to plunge height
            ArmHeight(plungeHeight, 0.75);
            if (armDestSuccess) {liftButton = 'n';}
        }
        else {
            liftPower = 0;
        }
        if (yoinkMode == false) {
            step = 0;
            conePlow.setPosition(plowHigh);
        }

        //------------------------ Reach ----------------------------
        if (topLift.getCurrentPosition() <= poleShort - 20){ // if we are placing a cone, extend our reach
            reach.setPosition(reachOut);
        }
        else { // if we are not, retract our grabber
            reach.setPosition(reachIn);
        }

        //################################### YOINK MODE ##############################################

        if (gamepad2.right_bumper) { //Initiate YOINK MODE! (for single cone) "Make it so"
            yoinkMode = true;
            step = 0;
            coneNumber = 1;
        }
        else if (gamepad2.dpad_down) {
            yoinkMode = true;
            step = 0;
            coneNumber = 2;
        }
        else if (gamepad2.dpad_left) {
            yoinkMode = true;
            step = 0;
            coneNumber = 3;
        }
        else if (gamepad2.dpad_right) {
            yoinkMode = true;
            step = 0;
            coneNumber = 4;
        }
        else if (gamepad2.dpad_up) {
            yoinkMode = true;
            step = 0;
            coneNumber = 5;
        }
        if (yoinkMode) {yoink();}

        leftFront.setPower(-leftFrontPower);
        rightFront.setPower(-rightFrontPower);

        leftRear.setPower(-leftRearPower);
        rightRear.setPower(-rightRearPower);

        topLift.setPower(-liftPower);
        bottomLift.setPower(liftPower);

        if (gamepad2.left_bumper){
            grabber.setPosition(openGrabber);
        }

        cycles = cycles +1;

        //############################# DRIVER FEEDBACK ###############################
        /*
        if (Dist_Intake.getDistance(DistanceUnit.MM) > 70 && Dist_Intake.getDistance(DistanceUnit.MM) < 180 && lift.getCurrentPosition() > poleShort)
        {
            gamepad1.rumbleBlips(1);
            gamepad2.rumbleBlips(1);
            gamepad2.setLedColor(1,100,1,500);
        }

        if (timeLeft >= 45) {
            // YES BLUE BLUE BLUE
            //are we in yoinking?
            if (!yoinkMode)
            {
                //no but the colors are pretty
              pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES;
              blinkinLedDriver.setPattern(pattern);
            }
            if (yoinkMode)
            {
                //YES ALERT EVERYONE AAAAA
                pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
                blinkinLedDriver.setPattern(pattern);
            }
        }
        */

        //is it endgame?
        if (timeLeft <= 30) {
            //are we yoinking?
            if (!yoinkMode)
            {
                //nope that's ok I guess
                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                blinkinLedDriver.setPattern(pattern);
            }
            if (yoinkMode)
            {
                //YES HAHAHHA LETS GOOOOO
                pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
                blinkinLedDriver.setPattern(pattern);
            }
        }

        // 15 second End Game warning lights
        // is it 15 sec to endgame?
        else if (timeLeft <= 40) {
            // YES YELLOW ALERT
            // are we yoinking?
            if (!yoinkMode) {
                // no dang man get it together
                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                blinkinLedDriver.setPattern(pattern);
            }
            else if(yoinkMode) {
                // YES BLIND THEM AND GO
                pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
                blinkinLedDriver.setPattern(pattern);
            }
        }

        if (gamepad2.share) { //turn telemetry on and off
            if (displaytelementry == false && cycles >= 50) {
                displaytelementry = true;
                cycles = 0; }
            else if (cycles >= 10) {
                //telemetry.clear();
                telemetry.update();
                displaytelementry = false;
                cycles = 0;
            }
        }
    }

    //----------------------------------------------------------------------------------------------------
    // FUNCTIONS (MYBLOCKS)
    //----------------------------------------------------------------------------------------------------
    private void yoink() {
        if (coneNumber == 1)
            grabHeight = 40;
        else if (coneNumber == 2)
            grabHeight = coneStack2;
        else if (coneNumber == 3)
            grabHeight = coneStack3;
        else if (coneNumber == 4)
            grabHeight = coneStack4;
        else if (coneNumber == 5) {
            grabHeight = coneStack5;
        }
        if (step == 0) {
            conePlow.setPosition(plowLow);
            grabber.setPosition(openGrabber); //close the grabber to get ready to grab a cone
            step = 1;
            cycles = 0;
        }

        // OLD CODE?
        /*
        if (step == 1 && yoinkMode && cycles >= 2)  //set the boom height to be ready to grab a cone
        {
            ArmHeight(plungeHeight, 0.5);
            if (armDestSuccess)
                step =2;
        }

        if (step == 2 && yoinkMode) { // Now move the arm UP until the distance sensor sees the servo
            if (distForZero.getDistance(DistanceUnit.MM) > 80)
            {
                liftPower = -0.6; //raise to distance sensor
            }
            else { //reached distance sensor, move motors back to encoder drive
                step = 3;
                liftPower = 0;
            }
        }
        */

        if (step == 1 && yoinkMode) //move up to grab height for cones 2-5 if grabbing from stack
        {
            ArmHeight(grabHeight, 0.8);
            if (armDestSuccess) {
                step = 4;
                cycles = 0;
                seeCone = false;
            }
        }

        // Step 4 - Ready to grab a cone, head unit is at correct height and distance sensors see cone, time to PARTY!
        //  if (step==4 && yoinkMode == true && cycles >= 3 && Dist_Intake.getDistance(DistanceUnit.MM) <= 85 && Dist_Intake.getDistance(DistanceUnit.MM) >= 60 )   // do we see a cone in front of the robot?

        if (step == 4 && yoinkMode == true && cycles >= 3 && ((DistanceSensor) frontColorDist).getDistance(DistanceUnit.MM) <= 200)    // do we see a cone in front of the robot?
            if ((distIntake.getDistance(DistanceUnit.MM) >= 47 && coneNumber == 1 && distIntake.getDistance(DistanceUnit.MM) <= 67) || (distIntake.getDistance(DistanceUnit.MM) >= 30 && coneNumber >= 2 && distIntake.getDistance(DistanceUnit.MM) <= 67)) {// is the cone aligned left to right based on cone height?
                seeCone = true;
            }

        if (step >= 4 && seeCone && yoinkMode) {
            if (step == 4) {
                ArmHeight(grabHeight + plungeHeight, 0.7);
                if (armDestSuccess) {step = 5;}
            }

            if (step == 5 && yoinkMode) {
                grabber.setPosition(closeGrabber);    // yoink that cone
                cycles = 0;
                step = 6;
            }

            if (step == 6 && cycles >= 6 && yoinkMode) {step = 7;}    // give grabber time to engage

            if (step == 7 && yoinkMode) {
                ArmHeight(330, 0.8);
                if (armDestSuccess) {
                    liftPower = 0;
                    yoinkMode = false;
                    seeCone = false;
                    conePlow.setPosition(plowHigh); //raise plow
                }
            }
        }
    }

    private void ArmHeight (double height, double power) {
        topLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (topLift.getCurrentPosition() - tolerance <= height && topLift.getCurrentPosition() + tolerance >= height) {
            liftPower = 0;
            armDestSuccess = true;
        }
        else if (topLift.getCurrentPosition() < height && topLift.getCurrentPosition() <= poleHigh)    // need to go up
        {
            armDestSuccess = false;
            if (topLift.getCurrentPosition() + 100 >= height) {liftPower = -0.6;}   // slow the arm down to prevent over shooting
            else {liftPower = -power;}
        }
        else if (topLift.getCurrentPosition() > height && lowerLimit.getState() == true && topLift.getCurrentPosition() >= plungeHeight)    // need to go down but don't hit lower limit switch
        {
            if (topLift.getCurrentPosition() - 500 <= height) {liftPower = 0.25;}    // slow the arm down to prevent over shooting
            else {liftPower = power;}

            armDestSuccess = false;
        }
    }

    public void MoveArm (int power, int height) {
        topLift.setTargetPosition(startHeight);
        bottomLift.setTargetPosition(-startHeight);
        liftPower = power;
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

    private void sendTelemetry() {
        telemetry.addData("---------- Luxo", "Telemetry ----------");
        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", desiredCourse, heading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", corrHeading, diffCorrection);

        //telemetry.addLine().

        telemetry.addData("Actual Pos Front L:R",  "%7d:%7d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
        telemetry.addData("Actual Pos Rear  L:R",  "%7d:%7d", leftRear.getCurrentPosition(),  rightRear.getCurrentPosition());

        //telemetry.addLine();

        telemetry.addData("Actual Pos Para:Perp",  "%7d:%7d", parallelEncoder.getCurrentPosition(), perpendicularEncoder.getCurrentPosition());

        //telemetry.addLine();

        telemetry.addData("Actual Pos topLift:BottomLift",  "%7d:%7d", topLift.getCurrentPosition(), bottomLift.getCurrentPosition());

        //telemetry.addLine();

        telemetry.addData("Wheel Speeds Front L:R", "%5.2f : %5.2f", leftFrontPower, rightFrontPower);
        //telemetry.addData("Wheel Speeds Rear  L:R", "%5.2f : %5.2f", leftRearPower,  rightRearPower);

        //telemetry.addLine();

        //telemetry.addData("distForZero", distForZero.getDistance(DistanceUnit.MM));
        //telemetry.addData("frontColorSensor", ((DistanceSensor) frontColorDist).getDistance(DistanceUnit.MM));
        //telemetry.addData("distIntake", distIntake.getDistance(DistanceUnit.MM));

        //telemetry.addLine();

        telemetry.addData("rangeSensor", rangeSensor.getDistance(DistanceUnit.CM));

        //telemetry.addLine();

        //telemetry.addData("Cycles", "%.3f", cycles);

        telemetry.update();
    }

    @Override
    public void stop() {}

}
