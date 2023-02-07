/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Archive;

import android.view.View;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Disabled
@Autonomous(name="FibbyAuto", group="Auto", preselectTeleOp = "FibbyTeleOp")
public class FibbyAuto extends LinearOpMode {
    // light controls variables
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    // declaring variables
    public DcMotor  leftFront   = null;
    public DcMotor  leftRear    = null;
    public DcMotor  rightFront  = null;
    public DcMotor  rightRear   = null;

    public DcMotor parallelEncoder;
    public DcMotor perpendicularEncoder;

    public DcMotor lift  = null;
    public DcMotor lift2 = null;

    public Servo grabber = null;

    public Servo DeadWheel_Lift;
    public Servo plow;

    //private DistanceSensor Dist_Intake;
    private DistanceSensor Dist_Intake2;
    NormalizedColorSensor FrontCDist;
    View relativeLayout;

    private DistanceSensor Dist_For_Zero;

    DigitalChannel lowerLimit;
    DigitalChannel upperLimit;

    private double leftFrontPower;
    private double leftRearPower;
    private double rightFrontPower;
    private double rightRearPower;

    double tickToINCH = 1068; //reading of the encoder per inch of motion
    int coneImage  = 0;

    boolean rightSide;
    //boolean forward;

    double plowlow = 0.28;
    double plowhigh = 0.24;

    double grabberopen = 0.03;
    double grabberclosed = 0.083;

    int plungeHeight = -140;
    int startHeight = 40;
    int poleShort = 470;
    int poleMid = 810;
    int poleHigh = 1230;
    int zero = 20;
    int tolerance = 10;

    double RaisePower = 0.9;
    double LowerPower = 0.6;

    int stack5 = 220;
    int stack4 = 165;
    int stack3 = 130;
    int stack2 = 120;
    int grabheight = 150;

    int LowerConeDist = 30;
    int UpperConeDist = 45;
    int FrontConeDist = 200; // was 150

    private       double corrHeading;
    private final double corrFactor = 0.03;
    private       double diffCorrection;

    double heading = 0;

//    BNO055IMU imu;
    IMU imu;

   // Orientation angles;
   // Acceleration gravity;

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

    //----------------------------------------------------------------------------------------------------
    // Start of the Auto
    //----------------------------------------------------------------------------------------------------
    @Override
    public void runOpMode()
    {
        // LED LIGHTS
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        imu = hardwareMap.get(IMU.class, "imu");

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear  = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        parallelEncoder      = hardwareMap.get(DcMotor.class, "parallelEncoder");
        perpendicularEncoder = hardwareMap.get(DcMotor.class, "perpendicularEncoder");

        //Dist_Intake  = hardwareMap.get(DistanceSensor.class, "dist_intake");

        FrontCDist = hardwareMap.get(NormalizedColorSensor.class, "dist_intake");
        Dist_Intake2 = hardwareMap.get(DistanceSensor.class, "dist_intake2");
        Dist_For_Zero = hardwareMap.get(DistanceSensor.class, "dist_for_zero");

        parallelEncoder.setDirection(DcMotor.Direction.REVERSE);
        perpendicularEncoder.setDirection(DcMotor.Direction.REVERSE);

        lift  = hardwareMap.get(DcMotor.class,"Lift");
        lift2 = hardwareMap.get(DcMotor.class,"Lift2");

        lift.setDirection(DcMotor.Direction.REVERSE);
        lift2.setDirection(DcMotor.Direction.REVERSE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        grabber = hardwareMap.get(Servo .class,"grabber");
        grabber.setPosition(grabberopen);

        plow = hardwareMap.get(Servo.class, "plow");
        plow.setPosition(plowlow);

        DeadWheel_Lift = hardwareMap.get(Servo.class,"Deadwheel_Lift");
        //DeadWheel_Lift.setPosition(0.5);

        lowerLimit = hardwareMap.get(DigitalChannel.class, "l_limit");
        upperLimit = hardwareMap.get(DigitalChannel.class, "u_limit");

        lowerLimit.setMode(DigitalChannel.Mode.INPUT);
        upperLimit.setMode(DigitalChannel.Mode.INPUT);

        while (lowerLimit.getState() == true){
            lift.setPower(-0.25);
            lift2.setPower(0.25);
        }
        while (Dist_For_Zero.getDistance(DistanceUnit.MM) > 80) {
            lift.setPower(0.4);
            lift2.setPower(-0.4);
        }

        lift.setPower(0);
        lift2.setPower(0);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
/*
        BNO055IMU.Parameters gyro_parameters = new BNO055IMU.Parameters();

        gyro_parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro_parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro_parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyro_parameters.loggingEnabled = true;
        gyro_parameters.loggingTag = "IMU";
        gyro_parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyro_parameters);
*/
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.0, 16.0/9.0);
        }

        Boolean questionAnswered = false;

        telemetry.addLine("Left Side - □ | Right Side - O");
        telemetry.update();

        while (questionAnswered == false) {
            if (gamepad1.b || gamepad2.b) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
                blinkinLedDriver.setPattern(pattern);
                rightSide = true;
                questionAnswered = true;
            }
            else if (gamepad1.x || gamepad2.x) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
                blinkinLedDriver.setPattern(pattern);
                rightSide = false;
                questionAnswered = true;
            }
            else {
                questionAnswered = false;
            }

            if (questionAnswered == true) {
                telemetry.clear();
                telemetry.update();
            }
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
            if (gamepad2.dpad_up|| gamepad1.dpad_up) {
                plow.setPosition(plowlow);
                liftENC(plungeHeight, -0.5);
                sleep(1000);
            }
            if (lift.getCurrentPosition() <= -100 && grabber.getPosition() != grabberclosed){
                lift.setPower(0);
                lift2.setPower(0);
                grabber.setPosition(grabberclosed);
                sleep(1000);
                liftENC(startHeight -40, 0.75);
                sleep(750);
                lift.setPower(0);
                lift2.setPower(0);
                plow.setPosition(plowhigh);
            }
        }

        // turn off the camera
        if (tfod != null) {tfod.shutdown();}
        pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_1_ON_2;
        blinkinLedDriver.setPattern(pattern);

        //GyroDriveStack(0, 0.25, 0, 1);
        TestThang();
        //ConeDrop();




        // GyroDriveStack(120, 0.2, 0, 1);
        //GyroSpin(0.3, 82);
        //sleep(500);
        //GyroSpin(0.25, -90);
        //GyroDriveENC(35,0.2, -90);
    }

    //----------------------------------------------------------------------------------------------------
    // Camera & Tensor Flow Initialization
    //----------------------------------------------------------------------------------------------------
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
    // Get Gyro Angle
    //----------------------------------------------------------------------------------------------------
    private void checkOrientation()
    {
        // read the orientation of the robot
     //   angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
     //   this.imu.getPosition();
        // and save the heading
     //   heading = -angles.firstAngle;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        heading = -orientation.getYaw(AngleUnit.DEGREES);
       // telemetry.addData("heading: ", "%s", heading);
       // telemetry.update();
    }

    //----------------------------------------------------------------------------------------------------
    // "My Blocks"
    //----------------------------------------------------------------------------------------------------
    public void GyroDriveENC(double distance, double power, double course, boolean reset, boolean forward)
    {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      if (reset == true){
           parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      }

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        power = Math.abs(power); //this ensures that if we are moving forward it is positive
        if (!forward){ // and if we are going backwards it is negative
            power = -power;
        }

        while ((parallelEncoder.getCurrentPosition()/tickToINCH < distance && forward && opModeIsActive())
                ||
                (parallelEncoder.getCurrentPosition()/tickToINCH > distance && !forward && opModeIsActive())) {
            checkOrientation();

            // it will make input power always positive
            //power = Math.abs(power);

            corrHeading = course - heading;
            diffCorrection = Math.abs(corrHeading * corrFactor);

            if (corrHeading < 0) { //robot is drifting to the right, so we need to correct to the left
                leftFrontPower = (power - diffCorrection);
                leftRearPower = (power - diffCorrection);
                rightFrontPower = (power + diffCorrection);
                rightRearPower = (power + diffCorrection);
            }
            else {
                leftFrontPower = (power + diffCorrection);
                leftRearPower = (power + diffCorrection);
                rightFrontPower = (power - diffCorrection);
                rightRearPower = (power - diffCorrection);
            }

            if ((Math.abs(distance) - Math.abs(parallelEncoder.getCurrentPosition()/tickToINCH)) <= 3) {
                if (power > 0.2) {

                }
            }

            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);
            telemetry.addLine()
                    .addData("Heading, Course, diffCorrection, power, corrHeading", "%s, %s, %.01f, %.01f, %s",
                            heading, course, diffCorrection, power, corrHeading);
            telemetry.addLine()
                    .addData("Encoder - Encoder/Inches", "%7d, %.01f",
                            parallelEncoder.getCurrentPosition(), parallelEncoder.getCurrentPosition()/tickToINCH);
           if (corrHeading <0)
            telemetry.addLine()
                            .addData("Turn Left - LeftPower, RightPower", "%5.2f, %5.2f",
                    power - diffCorrection, power);
            else
                telemetry.addLine()
                        .addData("Turn Right - LeftPower, RightPower", "%5.2f, %5.2f",
                                power, power - diffCorrection);


            telemetry.update();
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

    }

    public void GyroDriveStack(double distance, double power, double course, int stack) {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        plow.setPosition(plowlow);
/*
        if (stack == 5)
            liftENC(stack5, -0.5);
        else if (stack == 4)
            liftENC(stack4, -0.5);
        else if (stack == 3)
            liftENC(stack3, -0.5);
        else if (stack == 2)
            liftENC(stack2, -0.5);
        else if (stack == 1) {
            LowerConeDist = 52;
            UpperConeDist = 64;
            liftENC(startHeight + 10, -0.5);
        }

 */


//stack == 1 &&  Dist_Intake2.getDistance(DistanceUnit.MM) >= 30 && Dist_Intake2.getDistance(DistanceUnit.MM) <= 50)  //Single Cone
//               || // OR
        //while ((((DistanceSensor) FrontCDist).getDistance(DistanceUnit.MM) > FrontConeDist) && (opModeIsActive())) {
        while ((stack == 1 &&  (Dist_Intake2.getDistance(DistanceUnit.MM) <= 50 || Dist_Intake2.getDistance(DistanceUnit.MM) >= 67))  //Single Cone
               || // OR
                (stack >= 2 && (Dist_Intake2.getDistance(DistanceUnit.MM) <= 30 || Dist_Intake2.getDistance(DistanceUnit.MM) >= 50))// Stack of Cones
                && (opModeIsActive())) {
            checkOrientation();

            // it will make input power always positive
            //power = Math.abs(power);

            corrHeading = course - heading;
            diffCorrection = Math.abs(corrHeading * corrFactor);

            if (corrHeading < 0) {
                leftFrontPower = (power - diffCorrection);
                leftRearPower = (power - diffCorrection);
                rightFrontPower = (power);
                rightRearPower = (power);
            } else {
                leftFrontPower = (power);
                leftRearPower = (power);
                rightFrontPower = (power - diffCorrection);
                rightRearPower = (power - diffCorrection);
            }


            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);

        }

        /*
            while (Dist_Intake2.getDistance(DistanceUnit.MM) < LowerConeDist) {
                leftFront.setPower(-0.3);
                leftRear.setPower(0.3);
                rightFront.setPower(0.3);
                rightRear.setPower(-0.3);

            }


            while (Dist_Intake2.getDistance(DistanceUnit.MM) > UpperConeDist) {
                leftFront.setPower(0.3);
                leftRear.setPower(-0.3);
                rightFront.setPower(-0.3);
                rightRear.setPower(0.3);

            }

         */
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);


            if (stack == 5) {liftENC(stack5 - 110, -0.9);}
            else if (stack == 4) {liftENC(stack4 - 110, -0.6);}
            else if (stack == 3) {liftENC(stack3 -110, -0.6);}
            else if (stack == 2) {liftENC(stack2 - 150, -0.6);}
            else if (stack == 1) {liftENC(plungeHeight, 0.6);}

            sleep(400);

           /* GyroSpin(0.25, course+1);
            sleep(250);
            GyroSpin(0.25, course-1 );
            sleep(250);
            */

            grabber.setPosition(grabberclosed);
            sleep(300);

            liftENC(280, 0.75);
            plow.setPosition(plowhigh);
            sleep(750);
    }

    public void GyroStrafeENC(double distance, double power, String direction, double course)
    {
        distance = Math.abs(distance);
        //direction = toString().toLowerCase(direction);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (Math.abs(perpendicularEncoder.getCurrentPosition()/tickToINCH) <= distance && (opModeIsActive())) {
            checkOrientation();

            // it will make input power always positive
            //power = Math.abs(power);

            corrHeading = course - heading;
            diffCorrection = Math.abs(corrHeading * corrFactor);

            if ((corrHeading < 0 && direction == "left") || (corrHeading > 0 && direction == "right")) {
                leftFrontPower = (power);
                leftRearPower = (power - diffCorrection);
                rightFrontPower = (power);
                rightRearPower = (power - diffCorrection);
            }
            else {
                leftFrontPower = (power - diffCorrection);
                leftRearPower = (power);
                rightFrontPower = (power - diffCorrection);
                rightRearPower = (power);
            }

            if(direction == "left") {
                leftFront.setPower(-leftFrontPower);
                leftRear.setPower(leftRearPower);
                rightFront.setPower(rightFrontPower);
                rightRear.setPower(-rightRearPower);
            }
            else if(direction == "right") {
                leftFront.setPower(leftFrontPower);
                leftRear.setPower(-leftRearPower);
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

    public void GyroSpin(double power, double course) {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        checkOrientation();

        corrHeading = course - heading;
        //diffCorrection = Math.abs(corrHeading * corrFactor);
        if (heading >= course) {
            while (heading >= course) {
                leftFrontPower = (-power);
                leftRearPower = (-power);
                rightFrontPower = (power);
                rightRearPower = (power);

                leftFront.setPower(leftFrontPower);
                leftRear.setPower(leftRearPower);
                rightFront.setPower(rightFrontPower);
                rightRear.setPower(rightRearPower);
                checkOrientation();
            }
        }
        else if (heading <= course) {
            while (heading <= course) {
                leftFrontPower = (power);
                leftRearPower = (power);
                rightFrontPower = (-power);
                rightRearPower = (-power);

                leftFront.setPower(leftFrontPower);
                leftRear.setPower(leftRearPower);
                rightFront.setPower(rightFrontPower);
                rightRear.setPower(rightRearPower);
                checkOrientation();
            }
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    public void liftENC(int distance, double power) {
        if (distance > 1300) {
            distance = 1300;}
        /*}
        if ((power > 0 && lift.getCurrentPosition() <= distance) || ((power < 0 && lift.getCurrentPosition() >= distance + 15 && lowerLimit.getState() == true)) && (opModeIsActive())) {
            lift.setPower(power);
            lift2.setPower(-power);
        } else {
            lift.setPower(0);
            lift2.setPower(0);
        }*/

            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setTargetPosition(distance);
            lift2.setTargetPosition(-distance);
            lift.setPower(power);
            lift2.setPower(-power);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            /*if ((power < 0 && lift.getCurrentPosition() <= distance) || ((power > 0 && lift.getCurrentPosition() >= distance + 15 && lowerLimit.getState() == true)) && (opModeIsActive())) {
                lift.setPower(0);
                lift2.setPower(0);
            }*/
    }
    //----------------------------------------------------------------------------------------------------
    // Auto Runs
    //----------------------------------------------------------------------------------------------------
    public void Parking()
    {
        if (coneImage == 1) {
            telemetry.addData(">", "coneimage1");
            telemetry.update();
            //move left

            //move forward

        }
        else if (coneImage == 3) {
            telemetry.addData(">", "coneimage3");
            telemetry.update();
            //move forward


        }
        else {
            telemetry.addData(">", "coneimage2");
            telemetry.update();
            //move right

            //move forward
        }
    }
    public void TestThang(){
        GyroDriveENC(15, 0.5, 0, true, true);
        sleep(4000);
        GyroDriveENC(5, -0.5, 0, false, false);
        sleep(4000);
    /*public void ConeDrop()
    {
        String lSide = "left";
        String rSide = "right";

        if(!rightSide) {
            lSide = "right";
            rSide = "left";

        }

        // liftENC(startHeight, 0.85);

        // drive forward to push cone away and back up
        GyroDriveENC(47, 0.75, 0);
        sleep(400);                                 // sleep is important bc it allows time to complete drive foward
        GyroDriveENC(-2, -0.65, 0);
        sleep(400);

        // lift arm up and strafe to pole
        liftENC(poleHigh, 0.85);
        if (rightSide){
           GyroStrafeENC(10.5, 0.5, lSide,0);
        }
        else{
           GyroStrafeENC(10.5, 0.5, lSide,0);
        }

        // move close to pole
        if (rightSide) {
            GyroDriveENC(0.3, 0.3, 0);
            sleep(100);
        } else {
            GyroDriveENC(0.4, 0.3, 0);
            sleep(100);
        }

        // drop cone
        grabber.setPosition(grabberopen);
        sleep(100);

        // move backwards a little for only left side
        if (rightSide){}
        else {
            GyroDriveENC(-0.5, -0.45, 0);
        }

        // strafe to the stack of cones + lifting arm up
        GyroStrafeENC(11, 0.5, rSide, 0);
        liftENC(stack5, -0.5);
        sleep(250);

        // same movement but different for left and right
        if (rightSide) {
            // spin so we face the stack
            GyroSpin(0.5, 84);

            //drive towards stack and grab a cone
            GyroDriveENC(17,0.5, 86.5);
            GyroDriveStack(0, 0.25, 86.5, 5);   //picking up first cone

            //back up and lift arm to low height
            GyroDriveENC(-18, -0.6, 86.5);
            liftENC(poleShort, 1);

            //strafe to pole and open grabber
            GyroStrafeENC(9, 0.5, rSide, 86.5);
            GyroDriveENC(2, 0.7, 86.5);
            sleep(100);
            grabber.setPosition(grabberopen);
            sleep(100);

            //back away from pole while lifting arm up
            GyroDriveENC(-0.3, -0.3, 86.5);
            liftENC(stack4, -0.5);

            //strafe in front of pole and drive forward to grab
            GyroStrafeENC(9, 0.5, lSide, 86.5);
            GyroDriveENC(16,0.5, 86.5);
            GyroDriveStack(150, 0.25, 86.5, 4); //picking up second cone

            //back up and lift arm to low height
            GyroDriveENC(-17.5, -0.6, 86.5);
            liftENC(poleShort, 1);

            //strafe to pole and open grabber
            GyroStrafeENC(9, 0.5, rSide, 86.5);
            GyroDriveENC(2, 0.7, 86.5);
            sleep(150);
            grabber.setPosition(grabberopen);
            sleep(100);

            //back away from pole while lifting arm up
            GyroDriveENC(-0.3, -0.3, 86.5);
            liftENC(stack3, -0.5);

            //strafe in front of pole and drive forward to grab
            GyroStrafeENC(9, 0.5, lSide, 86.5);
            GyroDriveENC(16,0.5, 86.5);
            GyroDriveStack(150, 0.3, 86.5, 3);  //picking up third cone

            //back up and lift arm to low height
            GyroDriveENC(-17.5, -0.6, 86.5);
            liftENC(poleShort, 1);

            //strafe to pole and open grabber
            GyroStrafeENC(9, 0.5, rSide, 86.5);
            GyroDriveENC(2, 0.7, 86.5);
            sleep(150);
            grabber.setPosition(grabberopen);
            sleep(100);

            /*
            if (coneImage == 2 || coneImage == 0) {
                //back up and get ready to grab another code
                GyroDriveENC(-0.5, -0.3, 86.5);
                liftENC(stack2, -0.5);
                GyroStrafeENC(8.5, 0.5, lSide, 86.5);
                //drive towards the stack and grab another cone
                GyroDriveENC(16, 0.5, 86.5);
                GyroDriveStack(150, 0.4, 86.5, 2);
                //back up and deliver the cone
                GyroDriveENC(-18, -0.6, 86.5);
                liftENC(poleShort, 1);
                GyroStrafeENC(8.5, 0.5, rSide, 86.5);
                GyroDriveENC(2, 0.7, 86.5);
                sleep(100);
                grabber.setPosition(grabberopen);
                sleep(100);
            }
            */
            /*


        }
        else {
            //spin so we face the stack
            GyroSpin(0.5, -90);

            //drive towards stack and grab a cone
            GyroDriveENC(17,0.5, -90);
            GyroDriveStack(0, 0.25, -90, 5);    //picking up first cone

            //back up and lift arm to low height
            GyroDriveENC(-18, -0.6, -90);
            liftENC(poleShort, 1);

            //strafe to pole and open grabber
            GyroStrafeENC(10, 0.5, rSide, -90);
            GyroDriveENC(1, 0.7, -90);
            sleep(100);
            grabber.setPosition(grabberopen);
            sleep(100);

            //back away from pole while lifting arm up
            GyroDriveENC(-0.35, -0.3, -90);
            liftENC(stack4, -0.5);

            //strafe in front of pole and drive forward to grab
            GyroStrafeENC(9.5, 0.5, lSide, -90);
            GyroDriveENC(16,0.5, -90);
            GyroDriveStack(150, 0.25, -90, 4);  //picking up second cone

            //back up and lift arm to low height
            GyroDriveENC(-18, -0.6, -90);
            liftENC(poleShort, 1);

            //strafe to pole and open grabber
            GyroStrafeENC(10, 0.5, rSide, -90);
            GyroDriveENC(1.5, 0.7, -90);
            sleep(150);
            grabber.setPosition(grabberopen);
            sleep(100);

            //back away from pole while lifting arm up
            GyroDriveENC(-0.3, -0.3, -90);
            liftENC(stack3, -0.5);

            //strafe in front of pole and drive forward to grab
            GyroStrafeENC(10, 0.5, lSide, -90);
            GyroDriveENC(16,0.5, -90);
            GyroDriveStack(150, 0.3, -90, 3);   //picking up third cone

            //back up and lift arm to low height
            GyroDriveENC(-18.5, -0.6, -90);
            liftENC(poleShort, 1);

            //strafe to pole and open grabber
            GyroStrafeENC(10, 0.5, rSide, -90);
            GyroDriveENC(1.5, 0.7, -90);
            sleep(150);
            grabber.setPosition(grabberopen);
            sleep(100);

            /*
            if (coneImage == 2 || coneImage == 0) {
                //back up and get ready to grab another code
                GyroDriveENC(-0.5, -0.3, 86.5);
                liftENC(stack2, -0.5);
                GyroStrafeENC(8.5, 0.5, lSide, 86.5);
                //drive towards the stack and grab another cone
                GyroDriveENC(16, 0.5, 86.5);
                GyroDriveStack(150, 0.4, 86.5, 2);
                //back up and deliver the cone
                GyroDriveENC(-18, -0.6, 86.5);
                liftENC(poleShort, 1);
                GyroStrafeENC(8.5, 0.5, rSide, 86.5);
                GyroDriveENC(2, 0.7, 86.5);
                sleep(100);
                grabber.setPosition(grabberopen);
                sleep(100);
            }

             */
    /*

        }

        //liftENC(startHeight, -0.5);

        //set plow high
        plow.setPosition(plowhigh);

        // if camera see rose then do this
        if (coneImage == 1) {
            // strafe to zone 1
            if (rightSide) {
            GyroDriveENC(-1, -0.5, 86.5);
            liftENC(startHeight, -0.5);
            GyroStrafeENC(10, 0.75, "left", 86.5);
            GyroDriveENC(-17.5, -0.65, 90);
            }

            else {
                GyroDriveENC(-1, -0.5, -86.5);
                liftENC(startHeight, -0.5);
                GyroStrafeENC(7, 0.75, "left", -86.5);
                GyroDriveENC(18.5, 0.75, -90);
            }
            /*
            if (rightSide) {

                GyroStrafeENC(11, 0.5, "left", 0);
            }
            else {
                GyroStrafeENC(36, 0.5, "left", 0);
            }
            GyroDriveENC(-10, -0.4, 0);

             */
    /*
        }

        // if camera see pineapple then do this
        else if (coneImage == 3) {
            // strafe to zone 3
            //GyroStrafeENC(11, 0.5, rSide, 0);
            if (rightSide) {
                GyroDriveENC(-1, -0.5, 86.5);
                liftENC(startHeight, -0.5);
                GyroStrafeENC(10, 0.75, "right", 86.5);
                GyroDriveENC(18.5, 0.75, 90);
            }
            else {
                GyroDriveENC(-1, -0.5, -86.5);
                liftENC(startHeight, -0.5);
                GyroStrafeENC(10, 0.75, "right", -86.5);
                GyroDriveENC(-12.5, -0.65, -90);
            }
        }

        // if camera see snail then do this
        else {
            // stay in its currsent position
            if (rightSide) {
                GyroDriveENC(-1, -0.3, 86.5);
                liftENC(startHeight, -0.5);
            }
            else {
                GyroDriveENC(-1, -0.3, -86.5);
                liftENC(startHeight, -0.5);
            }
            /*
            if (rightSide) {
                GyroStrafeENC(36, 0.5, "right", 0);
            }
            else {
                GyroStrafeENC(11, 0.5, "right", 0);
            }
            liftENC(startHeight, -0.5);
            GyroDriveENC(-10, -0.4, 0);
        */
        //}

    //}

}
}