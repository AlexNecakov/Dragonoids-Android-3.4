/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Mecanum Drive", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class TeleOpMecanum extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor motorRF;
    DcMotor motorRB;
    DcMotor motorLF;
    DcMotor motorLB;

    DcMotor motorLift;
    DcMotor motorLift2;
    DcMotor motorRelic;

    //Declare servos
    Servo grabLeft;
    Servo grabRight;
    Servo juulKnocker;

    Servo relicArm;
    Servo relicGrab;

    //declaring various sensors
    ColorSensor colorSensor;
    DistanceSensor rangeSensor;
    BNO055IMU gyro;
    boolean color;

    boolean grabLock;
    boolean slowMode;
    boolean relicGrabbed;
    boolean relicLifted;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    double drive;
    double strafe;
    double rotate;


    final static int ENCODER_CPR = 1120;
    final static double WHEEL_CIRC = 4 * Math.PI;
    // 1 tile length is 24 inches
    final static int TILE = 24;

    final static double ROTATE = TILE / WHEEL_CIRC;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        motorRF = hardwareMap.dcMotor.get("right_drive_front");
        motorRB = hardwareMap.dcMotor.get("right_drive_back");
        motorLF = hardwareMap.dcMotor.get("left_drive_front");
        motorLB = hardwareMap.dcMotor.get("left_drive_back");

        motorLift = hardwareMap.dcMotor.get("lift");
        motorLift2 = hardwareMap.dcMotor.get("lift2");
        motorRelic = hardwareMap.dcMotor.get("relic");

        grabLeft = hardwareMap.get(Servo.class, "left_grabber");
        grabRight = hardwareMap.get(Servo.class, "right_grabber");

        grabLeft.setPosition(.5);
        grabRight.setPosition(.5);

        juulKnocker = hardwareMap.servo.get("jewel_knocker");
        juulKnocker.setPosition(1);

        relicArm = hardwareMap.servo.get("relicArm");
        relicGrab = hardwareMap.servo.get("relicGrab");

        colorSensor = hardwareMap.get(ColorSensor.class, "distanceColor");
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        rangeSensor = hardwareMap.get(DistanceSensor.class, "distanceColor");

        grabLock = false;
        slowMode = false;
        relicGrabbed = false;
        relicLifted = false;

        motorLF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorLB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motorLift.setDirection(DcMotor.Direction.REVERSE);
        motorLift2.setDirection(DcMotor.Direction.REVERSE);


//MediaPlayer player = MediaPlayer.create(hardwareMap.appContext, R.raw.file);
        //  player.start();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //output telemetry of motors
            telemetry.addData("rightFront", +motorRF.getPower());
            telemetry.addData("leftFront", +motorLF.getPower());
            telemetry.addData("rightBack", +motorRB.getPower());
            telemetry.addData("leftBack", +motorLB.getPower());

            drive = scaleInput(-gamepad1.left_stick_y);
            strafe = scaleInput(gamepad1.left_stick_x);
            rotate = scaleInput(gamepad1.right_stick_x);

            if(!slowMode) {
                motorLF.setPower(Range.clip(drive + strafe + rotate, -1.0, 1.0));
                motorLB.setPower(Range.clip(drive - strafe + rotate, -1.0, 1.0));
                motorRF.setPower(Range.clip(drive - strafe - rotate, -1.0, 1.0));
                motorRB.setPower(Range.clip(drive + strafe - rotate, -1.0, 1.0));
            }
            else{
                motorLF.setPower(.2*Range.clip(drive + strafe + rotate, -1.0, 1.0));
                motorLB.setPower(.2*Range.clip(drive - strafe + rotate, -1.0, 1.0));
                motorRF.setPower(.2*Range.clip(drive - strafe - rotate, -1.0, 1.0));
                motorRB.setPower(.2*Range.clip(drive + strafe - rotate, -1.0, 1.0));
            }

            if(gamepad1.a){
                slowMode = !slowMode;
            }


            if (gamepad2.right_stick_y > 0.4) {

                motorLift.setPower(.75);
                motorLift2.setPower(.75);
            }
            else if (gamepad2.right_stick_y < -0.4) {

                motorLift.setPower(-.75);
                motorLift2.setPower(-.75);
            }
            else if (Math.abs(gamepad2.right_stick_y) < 0.4) {
                motorLift.setPower(0);
                motorLift2.setPower(0);
            }


            /*if (gamepad2.left_stick_y > 0.4) {

                motorRelic.setPower(1);
            }
            else if (gamepad2.left_stick_y < -0.4) {

                motorRelic.setPower(-1);
            }
            else if (Math.abs(gamepad2.right_stick_y) < 0.4) {
                motorRelic.setPower(0);
            }



            if(relicLifted){
                relicArm.setPosition(1);
            }
            else{
                relicArm.setPosition(.2);
            }
            if(gamepad2.left_trigger>.3){
                relicLifted=!relicLifted;
            }

            if(!relicGrabbed){
                relicGrab.setPosition(0);
            }
            else{
                relicGrab.setPosition(1);
            }

            if(gamepad2.x){
                relicGrabbed=!relicGrabbed;
            }*/

            //theoretical lift code for going down constantly
           /*double liftPosition = ENCODER_CPR * ROTATE* gamepad2.left_trigger*.58;
            motorLift.setTargetPosition((int) liftPosition);
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift.setPower(.2);*/

            if (!grabLock) {
                if (gamepad2.right_trigger > 0.1) {

                    grabRight.setPosition(.8 + .2 * gamepad2.right_trigger);
                    grabLeft.setPosition(.2 - .2 * gamepad2.right_trigger);
                } else {
                    grabRight.setPosition(.8);
                    grabLeft.setPosition(.2);
                }
            }
            if (gamepad2.a){
                grabLock = !grabLock;
            }
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Distance Traveled: ", motorLF.getCurrentPosition() * (WHEEL_CIRC / ENCODER_CPR));
                telemetry.update();


        }

    }

    private float scaleInputOriginal(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.45, 0.52, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return (float)dScale;
    }
    private float scaleInput (double value) {
        // Return the value (from -1 to 1) squared to scale it quadratically
        float magnitude = (float) Math.pow(value, 2);
        if (value < 0) {
            return -1 * magnitude;
        }
        else {
            return magnitude;
        }
    }
    public boolean detectColor () {

        //false is red
        color = false;
        // convert the RGB values to HSV values.
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Blue ", colorSensor.blue());

        if (colorSensor.red()>colorSensor.blue()) {
            color = false;
        }
        else {
            color = true;
        }


        return color;
    }
    /*
package org.firstinspires.ftc.teamcode;

        import android.content.Context;
        import android.speech.tts.TextToSpeech;
        import android.util.Log;

        import java.util.Locale;

/**
 * Created by trucc on 3/1/2017.


public class RkrTTS {

    private static TextToSpeech tts;

    private static class TTSListener implements TextToSpeech.OnInitListener {

        @Override
        public void onInit(int initStatus) {
            if(initStatus == TextToSpeech.SUCCESS) {
                tts.setLanguage(Locale.US);
            } else {
                Log.d("RKR", "ERROR!");
            }
        }
    }

    public void init(Context context) {
        tts = new TextToSpeech(context, new TTSListener());
    }

    public void speakWords(String speech) {
        tts.speak(speech, TextToSpeech.QUEUE_ADD, null);
    }
}
 */
}
