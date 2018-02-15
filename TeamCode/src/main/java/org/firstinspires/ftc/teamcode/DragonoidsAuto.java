package org.firstinspires.ftc.teamcode;

//importing all relevant hardware classes
import android.graphics.Color;
import android.hardware.SensorEventListener;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import static java.lang.Thread.sleep;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Dragonoids on 11/5/2016.
 */

public class DragonoidsAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    //declaring all drive motors
    DcMotor motorRF;
    DcMotor motorRB;
    DcMotor motorLF;
    DcMotor motorLB;

    DcMotor motorLift;
    DcMotor motorLift2;

    //declaring servos
    Servo juulKnocker;
    Servo grabLeft;
    Servo grabRight;

    //declaring various sensors
    ColorSensor colorSensor;
    DistanceSensor rangeSensor;
    BNO055IMU gyro;
    ModernRoboticsI2cRangeSensor glyphSensor;
    DigitalChannel digitalTouch;

    //declaring vuforia instance
    VuforiaLocalizer vuforia;
    VuforiaTrackable relicTemplate;

    //declaring variables for angle adjustment
    public int targetAngle = 0;
    private int adjustedAngle;
    Orientation angles;
    int gyroAngle;

    // encoder counts per rotation on current wheels
    final static int ENCODER_CPR = 1120;
    //diameter of wheel is 2 inches
    final static double WHEEL_CIRC = 4 * Math.PI;
    // 1 tile length is 24 inches
    final static int TILE = 24;
    //number of rotations per tile
    final static double ROTATE = TILE / WHEEL_CIRC;

    //value to hold color being sensed
    int color;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};
    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    //runopmode is all of the functions that run when init is pressed on the robot
    public void runOpMode() throws InterruptedException {

        // get a reference to our various hardware objects. The string in the .get() method must be inputed into the phone config (case-sensitive)

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "gyro";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);

        gyroAngle = -(int)gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


        colorSensor = hardwareMap.get(ColorSensor.class, "distanceColor");
        rangeSensor = hardwareMap.get(DistanceSensor.class, "distanceColor");

        glyphSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "glyphSensor");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        motorRF = hardwareMap.dcMotor.get("right_drive_front");
        motorRB = hardwareMap.dcMotor.get("right_drive_back");
        motorLF = hardwareMap.dcMotor.get("left_drive_front");
        motorLB = hardwareMap.dcMotor.get("left_drive_back");

        grabLeft = hardwareMap.get(Servo.class, "left_grabber");
        grabRight = hardwareMap.get(Servo.class, "right_grabber");

        grabLeft.setPosition(.5);
        grabRight.setPosition(.5);

        juulKnocker = hardwareMap.servo.get("jewel_knocker");
        juulKnocker.setPosition(0);

        motorLift = hardwareMap.dcMotor.get("lift");
        motorLift2 = hardwareMap.dcMotor.get("lift2");

        //starts backwards and drives backwards
        motorLF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorLB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motorLift2.setDirection(DcMotor.Direction.REVERSE);

        //if drive motors receive no power, engage brakes
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        //LED is disabled as it does nothing relevant and draws power
        colorSensor.enableLed(false);

        //start vuforia with camera monitor
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //vuforia license DO NOT CHANGE THIS LINE
        vuParameters.vuforiaLicenseKey = "AegzGgb/////AAAAGY1xgpx1VUoDps1ud5K9VtYvynoEaO+Pg4uSUqw0ZTHIythxw9xKhy4+Ev14+mYsJAPNacxxg1TtpjWBVCHtvin9nRwZFMrgt086cfqtxBOrw+BHlj6tcy3oG33e/vCFmd755KLNFt8NbEM97YtYhJdrlxVKg7bZ4SJPl8QAu0XUrtjm/GlCz2GNrsIMYZ2ao6lMmfYzU/aUIRzGdBw46bZbFBNTXmbYB5Fml3jT9aKXDDSbH+HJHATrok7LO0+yd8Dbyhl/fLsRZ/vI1B/NRZNv/HpDIrEieDsyIDj60xV44BP4o3gR3URhgRamNxNl5ddZpxPqzB2xldScjENbF4ULBCLvIBOCudbI7BDiylrr";

        //activate rear camera
        vuParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vuParameters);

        //load Relic Recovery images
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.update();

        waitForStart();
        relicTrackables.activate();



    }

    //reset encoders needs to be called at the beginning of functions for distance to be accurately calculated. Makes encoder count zero again
    public void resetEncoders() {
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //stop motors needs to be called at the end of functions to avoid conflict. Turns off all drive motors
    public void stopMotors() {
        motorRF.setPower(0);
        motorRB.setPower(0);
        motorLF.setPower(0);
        motorLB.setPower(0);
    }

    /*auto corrects power to ensure correct horizontal/vertical motion
    CAN BE USED TO MAKE CURVED MOVEMENTS IF SLIGHTLY ITERATED. WAS UNABLE TO BE TESTED DUE TO SCOTT BEING THE NEW CHARLEY
    just create additional input to determine a target angle to adjust by instead of using the global target angle*/
    public void autoCorrect(double power, boolean motion){
        //true is forward false is strafe
        /*if(motion){
            motorRF.setPower(power+(targetAngle - gyroAngle) * .012);
            motorRB.setPower(power+(targetAngle - gyroAngle) * .012);
            motorLF.setPower(power-(targetAngle - gyroAngle) * .012);
            motorLB.setPower(power-(targetAngle - gyroAngle) * .012);
        }
        else if(!motion){
            motorRF.setPower(power+(targetAngle - gyroAngle) * .012);
            motorRB.setPower(power-(targetAngle - gyroAngle) * .012);
            motorLF.setPower(power+(targetAngle - gyroAngle) * .012);
            motorLB.setPower(power-(targetAngle - gyroAngle) * .012);
        }*/
    }

    //forward moves the robot forward passing a distance in units of tiles and a motor power
    public void forward (double distance, double power) {
        resetEncoders();

        //turn tile distance back into rotations
        distance = ENCODER_CPR * ROTATE * distance;

        //target position stores set number of rotations in encoder memory for use
        motorRF.setTargetPosition((int) distance);
        motorRB.setTargetPosition((int) distance);
        motorLF.setTargetPosition((int) distance);
        motorLB.setTargetPosition((int) distance);

        //turns on motors until they reach the target position
        motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //passes in specific power to drive motors
        motorRF.setPower(power);
        motorRB.setPower(power);
        motorLF.setPower(power);
        motorLB.setPower(power);

        //this loop actively corrects angle during movement by adjusting power based on gyro distance from target angle
        while (opModeIsActive() && (Math.abs(motorLB.getCurrentPosition())<=Math.abs(distance)-10 || Math.abs(motorRF.getCurrentPosition())<=Math.abs(distance)-10 ||
                Math.abs(motorRB.getCurrentPosition())<=Math.abs(distance)-10 || Math.abs(motorLF.getCurrentPosition())<=Math.abs(distance)-10)) {
                autoCorrect(power,true);
                telemetry.addData("Encoder Count LF: ", motorLF.getCurrentPosition());
                telemetry.addData("Encoder Count LB: ", motorLB.getCurrentPosition());
                telemetry.addData("Encoder Count RF: ", motorRF.getCurrentPosition());
                telemetry.addData("Encoder Count RB: ", motorRB.getCurrentPosition());
                gyroAngle = -(int)gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                telemetry.update();

        }
        telemetry.addData("Reached this line", true);
        telemetry.update();

        stopMotors();

    }

    //turn turns the robot on the spot passing in a specific angle in degrees. Can be positive or negative input
    public void turn (int angle) {
        resetEncoders();

        //no power is passed in input and is auto-calculated
        double power;

        //changes global target angle to the new angle to correct to that one
        targetAngle = angle;

        //R U E runs until different (non target position) criteria is met.
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //until gyro reaches target angle, pass power. Then break and finish
        if (targetAngle>0) {
            while (opModeIsActive() && (targetAngle > gyroAngle)) {

                gyroAngle = -(int)gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                if(targetAngle - gyroAngle>1 ){
                    power = -.8;
                }

                else{
                    break;
                }

                motorRF.setPower(power);
                motorRB.setPower(power);
                motorLF.setPower(-power);
                motorLB.setPower(-power);

                telemetry.addData("power" , power);
                telemetry.addData("Current angle" , gyroAngle);
                telemetry.addData("Target angle", targetAngle);
                telemetry.update();
            }
            stopMotors();
        }
        else {            while (opModeIsActive() && (targetAngle < gyroAngle)) {

                gyroAngle = -(int)gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                if(targetAngle - gyroAngle<-1) {
                    power = .8;
                }
                else {
                    break;
                }

                motorRF.setPower(power);
                motorRB.setPower(power);
                motorLF.setPower(-power);
                motorLB.setPower(-power);

                telemetry.addData("power" , power);
                telemetry.addData("Current angle" , gyroAngle);
                telemetry.addData("Target angle", targetAngle);
                telemetry.update();
            }
            stopMotors();
        }
        stopMotors();

        telemetry.addData("Angle reached", gyroAngle);
        telemetry.addData("Angle wanted", targetAngle);
        telemetry.update();

    }

    //strafe allows horizontal movement of the robot using mechanum drive. Positive distance is to the right, negative is to the left.
    public void strafe (double distance, double power) {
        resetEncoders();
        distance = ENCODER_CPR * ROTATE * distance;

        //motors in the direction of motion go inwards in mechanum drive, opposite wheels go outward.
        motorRF.setTargetPosition((int) -distance);
        motorRB.setTargetPosition((int) distance);
        motorLF.setTargetPosition((int) distance);
        motorLB.setTargetPosition((int) -distance);

        //runs until distance is reached
        motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRF.setPower(power);
        motorRB.setPower(power);
        motorLF.setPower(power);
        motorLB.setPower(power);

        //auto corrects based on distance from correct target angle
        while (opModeIsActive() && (Math.abs(motorLB.getCurrentPosition())<=Math.abs(distance)-10 || Math.abs(motorRF.getCurrentPosition())<=Math.abs(distance)-10 ||
                Math.abs(motorLF.getCurrentPosition())<=Math.abs(distance)-10 || Math.abs(motorRB.getCurrentPosition())<=Math.abs(distance)-10)) {

                    gyroAngle = -(int)gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                    autoCorrect(power,false);

        }

        stopMotors();

    }

    //right diagonal moves the robot in a right diagonal, passing distance and power as inputs
    public void rightDiagonal (double distance, double power) {
        resetEncoders();

        //tile to rotation calculation
        distance = ENCODER_CPR * ROTATE * distance * 2;

        //in mechanum drive only half the wheels need to move, the rest drag on the ground
        motorRF.setTargetPosition((int) distance);
        motorLB.setTargetPosition((int) distance);

        //runs until reaching set distance
        motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRF.setPower(power);
        motorLB.setPower(power);

        //wait until reaching target distance to stop
        while (opModeIsActive() && (Math.abs(motorRF.getCurrentPosition())<=Math.abs(distance) ||
                Math.abs(motorLB.getCurrentPosition())<=Math.abs(distance))) {
            
        }

        stopMotors();
    }

    //left diagonal moves the robot in a left diagonal, passing the distance and power as inputs
    public void leftDiagonal (double distance, double power) {
        resetEncoders();

        //tile back to rotation calculation
        distance = ENCODER_CPR * ROTATE * distance * 2;

        motorRB.setTargetPosition((int) distance);
        motorLF.setTargetPosition((int) distance);

        //wait until position is reached to stop
        motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRB.setPower(power);
        motorLF.setPower(power);

        while (opModeIsActive()&&(Math.abs(motorRB.getCurrentPosition())<=Math.abs(distance) || Math.abs(motorLF.getCurrentPosition())<=Math.abs(distance))) {
        }

        stopMotors();
    }

    //detect color interprets input from the color sensor and outputs a new color variable
    public int detectColor () {

        //0 is no color
        color = 0;
        // convert the RGB values to HSV values.
        Color.RGBToHSV((int) (colorSensor.red() * 255),
                (int) (colorSensor.green() * 255),
                (int) (colorSensor.blue() * 255),
                hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Blue ", colorSensor.blue());

        if (colorSensor.red()>colorSensor.blue()) {
            //red is 1
            color = 1;
        } else if (colorSensor.blue() > colorSensor.red()){
            //blue is 2
            color = 2;
        }

        return color;
    }


    public void knock(boolean team){

        juulKnocker.setPosition(1);
        sleep(900);
        //bool team is what team color you are, blue is false red is true
        if(team==true){

            if(detectColor()==1){
                forward(.1,.2);
                juulKnocker.setPosition(0.2);
                sleep(900);
                forward(-.1,.2);
            }
            else if(detectColor()==2){
                forward(-.1,.2);
                juulKnocker.setPosition(0.2);
                sleep(900);
                forward(.1, .2);
            }
        }
        else{

            if(detectColor()==1){
                forward(-.1,.2);
                juulKnocker.setPosition(0.2);
                sleep(900);
                forward(.1, .2);
            }
            else if(detectColor()==2){
                forward(.1,.1);
                juulKnocker.setPosition(0.2);
                sleep(900);
                forward(-.1, .18);
            }
        }


    }

    //target angle should return back to its target angle before the adjust
    public void adjustHeading() {

       int currentAngle = gyroAngle;

        int prevTargetAngle = targetAngle;

        adjustedAngle = (targetAngle);

        if (!(currentAngle > -8 && currentAngle < 8)) {
            turn(adjustedAngle);
        }
        targetAngle = prevTargetAngle;
    }


    public void alignLine(boolean value) {

        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(value) {
            while(false && opModeIsActive()){


                motorRF.setPower(-.15);
                motorRB.setPower(-.15);
                motorLF.setPower(-.15);
                motorLB.setPower(-.15); }
        } else {
            while(false && opModeIsActive()){

                motorRF.setPower(.15);
                motorRB.setPower(.15);
                motorLF.setPower(.15);
                motorLB.setPower(.15); }
            }
        stopMotors();
    }

    public double getRange () {
        double range = glyphSensor.getDistance(DistanceUnit.INCH)/24;
        return range;
    }

    public void adjustRange () {
        double range = getRange();
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (range>7) {
            while ((opModeIsActive()&&getRange()>7.75)) {
                motorRF.setPower(-.35);
                motorRB.setPower(.35);
                motorLF.setPower(.35);
                motorLB.setPower(-.35);
            }
        }
        else {
            while (opModeIsActive()&&(getRange()<7.75)){
                motorRF.setPower(.35);
                motorRB.setPower(-.35);
                motorLF.setPower(-.35);
                motorLB.setPower(.35);
            }
        }
        stopMotors();
        }

    public int photoSense() {
        int cryptokey = 0;

        long startTime = System.currentTimeMillis();

        while (cryptokey == 0&&(System.currentTimeMillis()-startTime)<1000) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark == RelicRecoveryVuMark.LEFT) {
                cryptokey= 1;
                telemetry.addData("VuMark", "%s visible", vuMark);
            } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                cryptokey= 2;
                telemetry.addData("VuMark", "%s visible", vuMark);
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                cryptokey= 3;
                telemetry.addData("VuMark", "%s visible", vuMark);
            } else {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();
        }
        return cryptokey;
    }

    public void chooseGlyph(int cryptokey, boolean team) {

        //blue = false, red = true
        if (team == false){
            if (cryptokey == 0) {
                //move to left glyph
                strafe(0.1, .5);
            }
            else if (cryptokey == 1) {
                //move to left glyph
                strafe(0.1, .5);
            } else if (cryptokey == 2) {
                //move to center glyph
                strafe(.45, .5);
            } else if (cryptokey == 3) {
                //move to right glyph
                strafe(.9, .5);
            }
        }
        else{
            if (cryptokey == 0) {
                //move to left glyph
                strafe(-0.1, .5);
            }
            else if (cryptokey == 1) {
                //move to left glyph
                strafe(-.9, .5);
            } else if (cryptokey == 2) {
                //move to center glyph
                strafe(-.45, .5);
            } else if (cryptokey == 3) {
                //move to right glyph
                strafe(-0.1, .5);
            }
        }
    }

    public void releaseGlyph(){

        grabLeft.setPosition(.5);
        grabRight.setPosition(.5);
        sleep(200);
        forward(-.2,.5);
        forward(.65,.5);
        forward(-.2,.5);
    }

    public void liftGlyph(){
        grabRight.setPosition(1);
        grabLeft.setPosition(0);
        sleep(300);
        motorLift.setPower(-1);
        motorLift2.setPower(-1);
        sleep(300);
        motorLift.setPower(0);
        motorLift2.setPower(0);
    }

    public void lowerGlyph(){
        grabLeft.setPosition(.5);
        grabRight.setPosition(.5);
        sleep(300);
        motorLift.setPower(1);
        motorLift2.setPower(1);
        sleep(300);
        motorLift.setPower(0);
        motorLift2.setPower(0);
    }

    public void multiGlyph(int column, boolean team){
        if(!team) {
            turn(90);
        }
        else{
            turn(-90);
        }
        resetEncoders();
        /*while(getRange()>1/24){
            motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorRF.setPower(.75);
            motorRB.setPower(.75);
            motorLF.setPower(.75);
            motorLB.setPower(.75);
        }*/

        forward(2,.9);
        double pileDist = motorLF.getCurrentPosition();
        int numFails = 0;

        while(digitalTouch.getState() == true&&runtime.time()<26){
            forward(1/12,.25);

            liftGlyph();
            if(digitalTouch.getState() == true) {
                lowerGlyph();
                numFails++;
            }
        }
        liftGlyph();
        pileDist = pileDist/(ENCODER_CPR*ROTATE);
        pileDist+=(numFails/12);

        forward(-pileDist,.9);

        if(!team) {
            turn(-90);
        }
        else{
            turn(90);
        }
        if(column==0&&!team){
            strafe(.45,.5);
        }
        else if(column==0&&team){
            strafe(-.45, .5);
        }
        else if(column==1){
            strafe(.45, .5);
        }
        else if(column==2){
            strafe(.45, .5);
        }
        else{
            strafe(-.45,.5);
        }
        forward(.4,.5);
        releaseGlyph();

    }
}
