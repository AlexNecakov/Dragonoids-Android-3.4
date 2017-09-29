package org.firstinspires.ftc.teamcode;

//importing all relevant hardware classes
import android.graphics.Color;
import android.hardware.SensorEventListener;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
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

    //declaring all drive motors
    DcMotor motorRF;
    DcMotor motorRB;
    DcMotor motorLF;
    DcMotor motorLB;

    //declaring various sensors
    ColorSensor colorSensor;
    DistanceSensor rangeSensor;
    ModernRoboticsI2cGyro gyro;

    //declaring vuforia instance
    VuforiaLocalizer vuforia;

    //declaring variables for angle adjustment
    public int targetAngle = 0;
    private int adjustedAngle;

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

        colorSensor = hardwareMap.get(ColorSensor.class, "distanceColor");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        rangeSensor = hardwareMap.get(DistanceSensor.class, "distanceColor");

        motorRF = hardwareMap.dcMotor.get("right_drive_front");
        motorRB = hardwareMap.dcMotor.get("right_drive_back");
        motorLF = hardwareMap.dcMotor.get("left_drive_front");
        motorLB = hardwareMap.dcMotor.get("left_drive_back");

        //starts backwards and drives backwards
        motorRF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorRB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        //if drive motors receive no power, engage brakes
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //gyro must be recalibrated upon every auto to ensure correct turns. Gyro is done calibrating when the DS outputs "calibrated:0"
        gyro.calibrate();
        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(1);
            telemetry.addData("Measurement mode", gyro.getMeasurementMode());
            telemetry.update();
        }
        gyro.resetZAxisIntegrator();

        // changed the heading to signed heading [-360,360]
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);

        //LED is disabled as it does nothing relevant and draws power
        colorSensor.enableLed(false);

        //start vuforia with camera monitor
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //vuforia license DO NOT CHANGE THIS LINE
        parameters.vuforiaLicenseKey = "AegzGgb/////AAAAGY1xgpx1VUoDps1ud5K9VtYvynoEaO+Pg4uSUqw0ZTHIythxw9xKhy4+Ev14+mYsJAPNacxxg1TtpjWBVCHtvin9nRwZFMrgt086cfqtxBOrw+BHlj6tcy3oG33e/vCFmd755KLNFt8NbEM97YtYhJdrlxVKg7bZ4SJPl8QAu0XUrtjm/GlCz2GNrsIMYZ2ao6lMmfYzU/aUIRzGdBw46bZbFBNTXmbYB5Fml3jT9aKXDDSbH+HJHATrok7LO0+yd8Dbyhl/fLsRZ/vI1B/NRZNv/HpDIrEieDsyIDj60xV44BP4o3gR3URhgRamNxNl5ddZpxPqzB2xldScjENbF4ULBCLvIBOCudbI7BDiylrr";

        //activate rear camera
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //load Relic Recovery images
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        //when starting the robot, target angle should be 0 and calibrate in the correct orientation
        telemetry.addData("Calibrated", targetAngle);
        telemetry.update();

        waitForStart();
        gyro.resetZAxisIntegrator();
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
        if(motion){
            motorRF.setPower(power+(targetAngle - gyro.getIntegratedZValue()) * .012);
            motorRB.setPower(power+(targetAngle - gyro.getIntegratedZValue()) * .012);
            motorLF.setPower(power-(targetAngle - gyro.getIntegratedZValue()) * .012);
            motorLB.setPower(power-(targetAngle - gyro.getIntegratedZValue()) * .012);
        }
        else if(!motion){
            motorRF.setPower(power+(targetAngle - gyro.getIntegratedZValue()) * .012);
            motorRB.setPower(power-(targetAngle - gyro.getIntegratedZValue()) * .012);
            motorLF.setPower(power+(targetAngle - gyro.getIntegratedZValue()) * .012);
            motorLB.setPower(power-(targetAngle - gyro.getIntegratedZValue()) * .012);
        }
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
        while (opModeIsActive() && (Math.abs(motorLF.getCurrentPosition())<=Math.abs(distance) || Math.abs(motorRF.getCurrentPosition())<=Math.abs(distance)/* ||
                Math.abs(motorLB.getCurrentPosition())<=Math.abs(distance) || Math.abs(motorLF.getCurrentPosition())<=Math.abs(distance))*/)) {
                autoCorrect(power,true);
        }

        stopMotors();

    }

    //turn turns the robot on the spot passing in a specific angle in degrees. Can be positive or negative input
    public void turn (int angle) {
        resetEncoders();

        //no power is passed in input and is auto-calculated
        double power;

        //changes global target angle to the new angle to correct to that one
        targetAngle = angle;

        //variable to keep track of current angle using gyro
        int currentAngle = gyro.getIntegratedZValue();

        //R U E runs until different (non target position) criteria is met.
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //until gyro reaches target angle, pass power. Then break and finish
        if ((targetAngle-currentAngle)>0) {
            while (opModeIsActive() && (targetAngle > gyro.getIntegratedZValue())) {

                if(((targetAngle - gyro.getIntegratedZValue()) * .009)> .4) {
                    power = .4;
                } else {
                    power = (targetAngle - gyro.getIntegratedZValue()) * .009;
                }

                if (power < .018) {
                    break;
                }

                motorRF.setPower(power);
                motorRB.setPower(power);
                motorLF.setPower(-power);
                motorLB.setPower(-power);

                telemetry.addData("power" , power);
                telemetry.addData("Current angle" , gyro.getIntegratedZValue());
                telemetry.addData("Target angle", targetAngle);
                telemetry.update();
            }
            stopMotors();
        }
        else {
            while (opModeIsActive() && (targetAngle < gyro.getIntegratedZValue())) {
                if(((targetAngle - gyro.getIntegratedZValue()) * .009)< -.4) {
                    power = -.4;
                } else {
                power = (targetAngle - gyro.getIntegratedZValue()) * .009;
                }

                if (power > -.018) {
                    break;
                }

                motorRF.setPower(power);
                motorRB.setPower(power);
                motorLF.setPower(-power);
                motorLB.setPower(-power);

                telemetry.addData("power" , power);
                telemetry.addData("Current angle" , gyro.getIntegratedZValue());
                telemetry.addData("Target angle", targetAngle);
                telemetry.update();
            }
            stopMotors();
        }
        stopMotors();

        telemetry.addData("Angle reached", gyro.getIntegratedZValue());
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
        while (opModeIsActive() && (Math.abs(motorRB.getCurrentPosition())<=Math.abs(distance) || Math.abs(motorRF.getCurrentPosition())<=Math.abs(distance)/* ||
                Math.abs(motorLB.getCurrentPosition())<=Math.abs(distance) || Math.abs(motorLF.getCurrentPosition())<=Math.abs(distance)*/)) {

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
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

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

    //target angle should return back to its target angle before the adjust
    public void adjustHeading() {

       int currentAngle = gyro.getIntegratedZValue();

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
        double range = rangeSensor.getDistance(DistanceUnit.INCH);
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

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        while (cryptokey == 0) {
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
}
