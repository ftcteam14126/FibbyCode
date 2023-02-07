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


import android.app.Activity;
import android.text.style.UpdateAppearance;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name="FibbyTeleOp", group="TeleOp")
public class FibbyTeleOp extends OpMode {
    //light controls variables
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    // declaring variables
    public DcMotor  leftFront   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  leftRear    = null;
    public DcMotor  rightRear   = null;

    private DcMotor parallelEncoder;
    private DcMotor perpendicularEncoder;

    public DcMotor lift = null;
    public DcMotor lift2 = null;
    public double liftPower = 0;

    public Servo grabber = null;
    public Servo getOutOfMyWay = null;

    public Servo reach = null;

    public Servo DeadWheel_Lift;
    public Servo plow;

    private DistanceSensor Dist_Intake;
    private DistanceSensor Dist_Intake2;
    NormalizedColorSensor FrontCDist;
    View relativeLayout;

    private DistanceSensor Dist_For_Zero;

    private ElapsedTime runtime = new ElapsedTime();

    DigitalChannel lowerLimit;
    DigitalChannel upperLimit;
    DigitalChannel led1_green;
    DigitalChannel led1_red;

    Boolean yoinkMode = false;
    Boolean seeCone = false;

    int step = 0;
    double plowlow = 0.32;
    double plowhigh = 0.24;
    double grabberopen = 0.083;
    double grabberclosed = 0.03;
    double reachIn = 0.47;
    double reachOut = 0;
    double timeLeft = 120;
    int plungeHeight = -140;
    int startHeight = 40;
    int poleShort = 470;
    int poleMid = 810;
    int poleHigh = 1230;
    int zero = 20;
    int tolerance = 10;
    double RaisePower = 0.9;
    double LowerPower = 0.6;
    double cycles = 0;
    char liftButton = 'n';
    double heading = 0;
    boolean displaytelementry = false;
    int stack5 = 220;
    int stack4 = 165;
    int stack3 = 130;
    int stack2 = 120;
    int grabheight = 150;
    boolean ArmDestSuccess = false;
    double conenumber =1;

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    //----------------------------------------------------------------------------------------------------
    // Initialization
    //----------------------------------------------------------------------------------------------------
    @Override
    public void init()
    {
        // LED LIGHTS
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        parallelEncoder      = hardwareMap.get(DcMotor.class, "parallelEncoder");
        perpendicularEncoder = hardwareMap.get(DcMotor.class, "perpendicularEncoder");
        //Dist_Intake  = hardwareMap.get(DistanceSensor.class, "dist_intake");

        FrontCDist = hardwareMap.get(NormalizedColorSensor.class, "dist_intake");

        Dist_Intake2 = hardwareMap.get(DistanceSensor.class, "dist_intake2");
        Dist_For_Zero = hardwareMap.get(DistanceSensor.class, "dist_for_zero");
        parallelEncoder.setDirection(DcMotor.Direction.REVERSE);
        perpendicularEncoder.setDirection(DcMotor.Direction.REVERSE);

        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift = hardwareMap.get(DcMotor.class,"Lift");
        lift.setDirection(DcMotor.Direction.REVERSE);

        lift2 = hardwareMap.get(DcMotor.class,"Lift2");
        lift2.setDirection(DcMotor.Direction.REVERSE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        grabber = hardwareMap.get(Servo.class,"grabber");
        grabber.setPosition(grabberopen);

        reach = hardwareMap.get(Servo.class, "reach");

        plow = hardwareMap.get(Servo.class, "plow");
        plow.setPosition(plowhigh);

        getOutOfMyWay = hardwareMap.get(Servo.class,"getoutofmyway");
        getOutOfMyWay.setPosition(0);

        DeadWheel_Lift = hardwareMap.get(Servo.class,"Deadwheel_Lift");
        DeadWheel_Lift.setPosition(0.25);

        lowerLimit = hardwareMap.get(DigitalChannel.class, "l_limit");
        upperLimit = hardwareMap.get(DigitalChannel.class, "u_limit");

        lowerLimit.setMode(DigitalChannel.Mode.INPUT);
        upperLimit.setMode(DigitalChannel.Mode.INPUT);

        led1_green = hardwareMap.get(DigitalChannel.class, "led1_green");
        led1_red = hardwareMap.get(DigitalChannel.class, "led1_red");
        led1_green.setMode(DigitalChannel.Mode.OUTPUT);
        led1_red.setMode(DigitalChannel.Mode.OUTPUT);

       /* while (lowerLimit.getState() == true){
            lift.setPower(-0.25);
            lift2.setPower(0.25);
        }
        while (Dist_For_Zero.getDistance(DistanceUnit.MM) > 80) {
            lift.setPower(0.4);
            lift2.setPower(-0.4);
        }*/

        lift.setPower(0);
        lift2.setPower(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        pattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;
        blinkinLedDriver.setPattern(pattern);

        BNO055IMU.Parameters gyro_parameters = new BNO055IMU.Parameters();
        gyro_parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro_parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro_parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyro_parameters.loggingEnabled = true;
        gyro_parameters.loggingTag = "IMU";
        gyro_parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyro_parameters);
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
    }

    //----------------------------------------------------------------------------------------------------
    // Initialization Loop
    //----------------------------------------------------------------------------------------------------
    @Override
    public void init_loop()
    {

    }

    //----------------------------------------------------------------------------------------------------
    // Start
    //----------------------------------------------------------------------------------------------------
    @Override
    public void start()
    {
    runtime.reset();
    DeadWheel_Lift.setPosition(0.215);
    }
    public void movearm (int power, int height) {
        lift.setTargetPosition(startHeight);
        lift2.setTargetPosition(-startHeight);
        liftPower = power;

}

    //----------------------------------------------------------------------------------------------------
    // Loop
    //----------------------------------------------------------------------------------------------------
    @Override
    public void loop() {
        timeLeft = 120 - runtime.seconds();
        double leftFrontPower;
        double leftRearPower;
        double rightFrontPower;
        double rightRearPower;



if (displaytelementry) {
    checkOrientation();

    telemetry.addData("leftFront", "%7d", leftFront.getCurrentPosition());
    telemetry.addData("rightFront", "%7d", rightFront.getCurrentPosition());
    telemetry.addData("parallelEncoder", "%7d", leftFront.getCurrentPosition());
    telemetry.addData("rightFront", "%7d", rightFront.getCurrentPosition());


    telemetry.addData("LF, RF, Parallel, Perpendicular, Lift, Lift2", "Starting at %7d :%7d :%7d :%7d :%7d :%7d",
            leftFront.getCurrentPosition(),
            rightFront.getCurrentPosition(),
            parallelEncoder.getCurrentPosition(),
            perpendicularEncoder.getCurrentPosition(),
            lift.getCurrentPosition(),
            lift2.getCurrentPosition());
    telemetry.addLine()
            .addData("Dist_For_Zero, Dist_Intake, Dist_Intake2", "%.01f mm, %.01f mm, %.01f mm",
                    Dist_For_Zero.getDistance(DistanceUnit.MM),
                    //Dist_Intake.getDistance(DistanceUnit.MM),
                    ((DistanceSensor) FrontCDist).getDistance(DistanceUnit.MM),
                    Dist_Intake2.getDistance(DistanceUnit.MM));
    telemetry.addLine()
            .addData("Cycles:", "%.3f", cycles);
    telemetry.update();
}

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
        } else if (gamepad1.left_stick_x != 0) {
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
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        //Test Code
        if (gamepad1.dpad_left){
            grabber.setPosition(grabberclosed);
        }
        if (gamepad1.dpad_right){
            grabber.setPosition(grabberopen);
        }



        //Limit switch - false == pressed, true == not pressed
        //This if statement is broken into two chunks, if the dri ver is trying to lower the lift AND the lower limit is NOT pressed,
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

        /*else if (gamepad2.dpad_left) {
            liftButton = 'l';
            yoinkMode=false;
        }*/

        // safety to make sure we don't break ourselves
        if ((lift.getCurrentPosition() >= poleHigh + 50) || (lowerLimit.getState() == true)) {
            liftPower = 0;
        }


        /*if ((gamepad2.right_stick_y > 0 && lowerLimit.getState() == true) || (gamepad2.right_stick_y < 0 && lift.getCurrentPosition() <= poleHigh+50))
        {

            if(gamepad2.right_stick_y < 0) {
                liftPower = gamepad2.right_stick_y;
            }
            else {
                liftPower = gamepad2.right_stick_y * .75;
            }
        }
       /* else {
            liftButton = 'n';
        }
*/




        if (gamepad2.right_stick_y != 0) {
            liftButton = 'n';
        } else if (liftButton == 'a') { //arm to drive height// {
          ArmHeight(startHeight, 0.75);
             if (ArmDestSuccess) {
                liftButton = 'n';
                liftPower = 0.0;
            }
        } else if (liftButton == 'b') { //arm to low junction
            {
                ArmHeight(poleShort, 0.85);
                if (ArmDestSuccess)
                    liftButton = 'n';
            }
        } else if (liftButton == 'x') { // arm to medium junction
            {
                ArmHeight(poleMid, 1);
                if (ArmDestSuccess)
                    liftButton = 'n';
            }
        } else if (liftButton == 'y')  //arm to high junction
            {
                ArmHeight(poleHigh, 1);
                if (ArmDestSuccess)
                    liftButton = 'n';
            }
         else if (liftButton == 'l')  //arm to plunge height
            {
                ArmHeight(plungeHeight, 0.75);
                if (ArmDestSuccess)
                    liftButton = 'n';

            }
         else {
            liftPower = 0;
        }
        if (yoinkMode == false) {
            step = 0;
            plow.setPosition(plowhigh);
        }

        if (gamepad2.share) { //turn telemetry on and off
            if (displaytelementry == false && cycles >= 50) {
                displaytelementry = true;
                cycles = 0; }
            else if (cycles >= 10) {
                telemetry.clearAll();
                telemetry.update();
                displaytelementry = false;
                cycles = 0;
            }
        }
        //------------------------ Reach ----------------------------

        if (lift.getCurrentPosition() <= poleShort){ // if we are placing a cone, extend our reach
            reach.setPosition(reachOut);
        }
        else { // if we are not, retract our grabber
            reach.setPosition(reachIn);
        }




        //################################### YOINK MODE ##############################################

        if (gamepad2.right_bumper) { //Initiate YOINK MODE! (for single cone) "Make it so"
            yoinkMode = true;
            step = 0;
            conenumber = 1;
        }
        else if (gamepad2.dpad_down) {
            yoinkMode = true;
            step = 0;
            conenumber = 2;
        }
        else if (gamepad2.dpad_left) {
            yoinkMode = true;
            step = 0;
            conenumber = 3;
        }else if (gamepad2.dpad_right) {
            yoinkMode = true;
            step = 0;
            conenumber = 4;
        }
        else if (gamepad2.dpad_up) {
            yoinkMode = true;
            step = 0;
            conenumber = 5;
        }
        if (yoinkMode)
            yoink();




        leftFront.setPower(-leftFrontPower);
        rightFront.setPower(-rightFrontPower);

        leftRear.setPower(-leftRearPower);
        rightRear.setPower(-rightRearPower);

        lift.setPower(-liftPower);
        lift2.setPower(liftPower);

        //telemetry.addData("Servo Pos", grabber.getPosition());
        if (gamepad2.left_bumper){
            grabber.setPosition(grabberopen);
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
            //YES YELLOW ALERT
            //are we yoinking?
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


    }


    private void yoink() {

        if (conenumber == 1)
            grabheight = 40;
        else if (conenumber == 2)
            grabheight = stack2;
        else if (conenumber == 3)
            grabheight = stack3;
        else if (conenumber == 4)
            grabheight = stack4;
        else if (conenumber == 5) {
            grabheight = stack5;
        }
        if (step == 0)
        {
            plow.setPosition(plowlow);
            grabber.setPosition(grabberopen); //close the grabber to get ready to grab a cone
            step = 1;
            cycles = 0;
         }
        /* if (step == 1 && yoinkMode && cycles >= 2)  //set the boom height to be ready to grab a cone
        {
            ArmHeight(plungeHeight, 0.5);
            if (ArmDestSuccess)
                step =2;
        }
        if (step == 2 && yoinkMode) { // Now move the arm UP until the distance sensor sees the servo
            if (Dist_For_Zero.getDistance(DistanceUnit.MM) > 80)
            {
                liftPower = -0.6; //raise to distance sensor
            }
            else { //reached distance sensor, move motors back to encoder drive
                step = 3;
                liftPower = 0;
            }

        }

         */

        if (step==1 && yoinkMode) //move up to grab height for cones 2-5 if grabbing from stack
        {
            ArmHeight(grabheight, 0.8);
            if (ArmDestSuccess)
            {
                step =4;
                cycles = 0;
                seeCone = false; }
            }


// Step 4 - Ready to grab a cone, head unit is at correct height and distance sensors see cone, time to PARTY!
      //  if (step==4 && yoinkMode == true && cycles >= 3 && Dist_Intake.getDistance(DistanceUnit.MM) <= 85 && Dist_Intake.getDistance(DistanceUnit.MM) >= 60 ) // Do we see a cone in front of the robot?

        if (step==4 && yoinkMode == true && cycles >= 3 && ((DistanceSensor) FrontCDist).getDistance(DistanceUnit.MM) <= 200) // Do we see a cone in front of the robot?
            if ((Dist_Intake2.getDistance(DistanceUnit.MM) >= 47 && conenumber == 1 && Dist_Intake2.getDistance(DistanceUnit.MM) <= 67) || (Dist_Intake2.getDistance(DistanceUnit.MM) >= 30 && conenumber >= 2 && Dist_Intake2.getDistance(DistanceUnit.MM) <= 67)) //Is the cone algined left to right based on cone height?
            seeCone = true;

        if (step >= 4 && seeCone && yoinkMode) {
            if (step == 4) {
                ArmHeight(grabheight + plungeHeight, 0.7);
                if (ArmDestSuccess)
                    step = 5;

            }

           if (step == 5 && yoinkMode) {
                grabber.setPosition(grabberclosed); // yoink that cone
                cycles =0;
                step = 6;
            }
            if (step == 6 && cycles >= 4 && yoinkMode) {
                step = 7;
            } // give grabber time to engage

           if (step == 7 && yoinkMode) {
               ArmHeight(300, 0.8);
                if (ArmDestSuccess) {
                    liftPower = 0;
                    yoinkMode = false;
                    seeCone = false;
                    plow.setPosition(plowhigh); //raise plow

                }
            }
        }}

    private void ArmHeight (double height, double power)
    {


        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       if (lift.getCurrentPosition() - tolerance <= height && lift.getCurrentPosition() +tolerance >= height) {
           liftPower = 0;
           ArmDestSuccess = true;
       }
       else if (lift.getCurrentPosition() < height && lift.getCurrentPosition() <= poleHigh) //need to go up
        {
            ArmDestSuccess = false;
            if (lift.getCurrentPosition() + 100 >= height)
                liftPower = -0.4; //Slow the arm down to prevent over shooting
            else
                liftPower=-power;
        }
        else if (lift.getCurrentPosition() > height && lowerLimit.getState() == true && lift.getCurrentPosition() >= plungeHeight) // need to go down but don't hit lower limit switch
        {
            if (lift.getCurrentPosition() - 250 <= height)
                liftPower = 0.25; //Slow the arm down to prevent over shooting
            else
                liftPower = power;
            ArmDestSuccess = false;
        }

    }

    private void checkOrientation()
    {
        // read the orientation of the robot
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        // and save the heading
        heading = -angles.firstAngle;

       //telemetry.addLine().addData("heading: ", "%s", heading);
        //telemetry.update();
    }
    @Override

    public void stop()
    {

    }
}
