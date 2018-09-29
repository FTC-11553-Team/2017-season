/* Copyright (c) 2017 FIRST. All rights reserved.
 */
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import android.graphics.Color;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

//import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
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

//@Autonomous(name="zBlue_NEAR_relicZone", group ="zzz")
@Disabled
public class zBlue_NEAR_relicZone_Orig extends Red_NEAR_relicZone {

    //DcMotor makes it drive forward or backward
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor armMotor;
    
    private static int TICK2INCHES = 120;

    // declare for Servo installed in the right
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   5;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo   servoLeft;
    Servo   servoArm;

    boolean rampUp = true;
    boolean forward = false;

    // declare variables for jewel color sensor
    ColorSensor jewelSensorColorLeft;
    //DistanceSensor jewelSensorDistance;

    // declare variables for pictograph reader
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    // create a timer
    private ElapsedTime runtime = new ElapsedTime();
    
        @Override public void runOpMode() {

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        // before waiting for init, check the hardware information - right Servo*/
        servoLeft = hardwareMap.get(Servo.class, "leftServo");
        servoArm = hardwareMap.get(Servo.class, "armServo");
        double  position = servoLeft.getPosition();
       if (position != 0.25)
       {
           servoLeft.setPosition(0.25);}
        position = servoLeft.getPosition();
        telemetry.addData("original ServoLeft is ", position);
        
        // tight arm Servo
        servoArm.setPosition(0.679);
        telemetry.addData("original ServoRight is ", servoArm.getPosition());
        telemetry.update();
     /* before waiting for init, check the hardware information - Sensor2*/
        // get a reference to the color sensor.
        jewelSensorColorLeft = hardwareMap.get(ColorSensor.class, "leftColorSensor");

        // get a reference to the distance sensor that shares the same name.
        //   jewelSensorDistance = hardwareMap.get(DistanceSensor.class, "rightColorSensor");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int colorrelativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View colorrelativeLayout = ((Activity) hardwareMap.appContext).findViewById(colorrelativeLayoutId);


        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            servoLeft.setPosition(0.85);
            sleep(1500);
            telemetry.addData("Servo Position", "%5.2f", servoLeft.getPosition());


            // after the beam is at the right place, read the jewel color information feedback
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (jewelSensorColorLeft.red() * SCALE_FACTOR),
                    (int) (jewelSensorColorLeft.green() * SCALE_FACTOR),
                    (int) (jewelSensorColorLeft.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();
            // sleep(5000);


            if (hsvValues[0]>150 && hsvValues[0]<250 ) // this is the case for the jewel is blue color, then robot move forward.
            {
                DriveDistance(0.5,0.5,-300);// talk to DC motor to move forward.

                forward = true;
            }
            else  // this is the case for the jewel is read color, then robot moves backward
            {
                DriveDistance(0.5,0.5,300);
                // talk to DC motor to move backward.

            }
            // move the beam back to original
            //  position = 0;
           
           servoRight.setPosition(0.95);
            sleep(CYCLE_MS);
          telemetry.addData("current reading is ",servoRight.getPosition() );
            armMotor.setPower(-1);
            sleep(200);
            armMotor.setPower(0);
    
            servoLeft.setPosition(0.25);
            sleep(CYCLE_MS);
            telemetry.addData("current reading is ",servoLeft.getPosition() );
            armMotor.setPower(1);
            sleep(200);
            armMotor.setPower(0);
            DriveDistance(0.5,0.5,3000);
            DriveDistance(0,1,1500);
            sleep(2000);
            servoArm.setPosition(0.58);
            DriveDistance(1,0,-1500);
            sleep(2000);
            
            
            // add more program to move more clyph if there is enough time.
            telemetry.addData("time used so far ", runtime.time());
            telemetry.update();
            if (runtime.time()<4)
            {
                // go to the safe zone.

            }

            telemetry.update();
            sleep(20000);
            // release block and move backforward.
             
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    public void DriveFoward (double powerLeft, double powerRight)
    {
        leftMotor.setPower(powerLeft);
        rightMotor.setPower(powerRight);
    }

    public void StopDriving()
    {
        DriveFoward(0,0);
    }
    //this code is to control the motor go forward, backword, turn left or right by setting the power and distance
    public void DriveDistance (double powerLeft, double powerRight, int distance)
    {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setTargetPosition(-distance);
        rightMotor.setTargetPosition(distance);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DriveFoward(powerLeft, powerRight);

        while(leftMotor.isBusy() && rightMotor.isBusy())
        {
        }
        StopDriving();
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
