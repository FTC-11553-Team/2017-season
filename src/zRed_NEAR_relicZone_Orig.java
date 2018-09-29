/* Copyright (c) 2017 FIRST. All rights reserved.
 */
package org.firstinspires.ftc.teamcode;
// Servo

import android.app.Activity;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

//@Autonomous(name="Red_NEAR_relicZone_Orig", group ="zzz")
@Disabled
public class zRed_NEAR_relicZone_Orig extends LinearOpMode {

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
    Servo   servoRight;
    Servo   servoArm;

    boolean rampUp = true;
    boolean forward = false;

    // declare variables for jewel color sensor
    ColorSensor jewelSensorColorRight;
    //DistanceSensor jewelSensorDistance;

    

    OpenGLMatrix lastLocation = null;

    // create a timer
    private ElapsedTime runtime = new ElapsedTime();
    
    @Override public void runOpMode() {

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        // before waiting for init, check the hardware information - right Servo*/
        servoRight = hardwareMap.get(Servo.class, "rightServo");
        servoArm = hardwareMap.get(Servo.class, "armServo");
        //
         double  position = servoRight.getPosition();
         if (position != 0.95)     {    servoRight.setPosition(0.95);}
        position = servoRight.getPosition();
        telemetry.addData("original ServoRight is ", position);
        // tight arm Servo
        servoArm.setPosition(0.679);
        telemetry.addData("original ServoRight is ", servoArm.getPosition());
        telemetry.update();
       /* before waiting for init, check the hardware information - Sensor2*/
        // get a reference to the color sensor.
        jewelSensorColorRight = hardwareMap.get(ColorSensor.class, "rightColorSensor");

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

        //int colorrelativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        //final View colorrelativeLayout = ((Activity) hardwareMap.appContext).findViewById(colorrelativeLayoutId);

     /* before waiting for init, check the hardware information - Camera*/
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
      //  int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        // This key is for the phone with faceplate broken.
        // parameters.vuforiaLicenseKey = "AanSy3j/////AAAAGQC7QG1oHEC3rVUohi6NIz8WcAtSz3IUWr5Tu0MWPIFd3fmTYPT7GB22IjYWoIB/HBXAezqIw70QFWi2fT7zG2cmTcXW7/C176GMtcDUlDpg3WCw0RRtMDSyJ8Zlz+wz0NZoOPLamuL7RVT9sGcfgs8mc1i0k/WztZWVOvVrLNj7jj/mVaqvQffnI+G9VxiB9RCkIFMnB/te79jDk5Mr7Euj+Vwa941x4f5k9ah59hHNGt2pgTBdvLHF9i4X20Fm4y++gTkkJqkXsd8ZA3n709OLMAGqsXJ9E9IGLZ57yT2xeuo+KtwDQ7D3LCqjd/Y/rfT65KS8cIJ7iLkdcdTvpfQzYKTI4wz0jhFoOwkDsPFY";
        // this key is for phone named 11553 spare ZTE 1501
       // parameters.vuforiaLicenseKey = "AQj4vSX/////AAAAGcbQTpRxTUnWmWG9QxUucrx3mKjFb2eOllJLhHE2pqQOEHD7TuHaoC0+OyIvJ6o07YfE5qQp/rVWC2kfhtd7NnSI9eKMPrCpmbjG2BS2suW2TYhzBIWNcg/3BGKqWYLOUZdsMQxuCUc/pcP3mZ6IbmGKXLvsNNBIfzoCed9uBREMOWQLCi+uagMUauB12BGp7FlCAeoG3y/1479/LxBWX1LQnHgjX7UTxCfd1BJptpzjxxJJIq/ZTAVUBi422If1TN4EPyGabzzLljCWxWd56i50N2oheSgltiAywmJ2EvRCB1ovBlusV9XVDjDLwOR3mn6/8fy2ErZ2DdCG14uUecV+3AMvGjLMqlzpuZX4cTfC";
        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
      //  parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        // parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
       // this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
     //   VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
     //   VuforiaTrackable relicTemplate = relicTrackables.get(0);
     //   relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
        runtime.reset();
        //relicTrackables.activate();
        if (opModeIsActive()) {
            // Read the pictograph first.
            /*----------------this part of program is to get pictograph data*/
            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
         // sleep(800);
         //   RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            
         //   telemetry.addData("wait for ",vuMark);
         //   telemetry.update();
         //   if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                // telemetry.addData("VuMark", "%s visible", vuMark);
                // telemetry.addData("VuMark is",vuMark);
         //       if (vuMark ==RelicRecoveryVuMark.CENTER){
         //           telemetry.addData("VuMark is",vuMark);
         //           telemetry.update();
          //      }
          //      else if(vuMark ==RelicRecoveryVuMark.LEFT){
          //          telemetry.addData("VuMark is",vuMark);
          //          telemetry.update();
          //      }
          //      else if(vuMark ==RelicRecoveryVuMark.RIGHT){
          //          telemetry.addData("VuMark is",vuMark);
          //          telemetry.update();
          //      }
           //     else
           //     {telemetry.addData("the result is unexpected, check the code",vuMark);}

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
              /*  OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                //      telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
               // if (pose != null) {
                 //   VectorF trans = pose.getTranslation();
                 //   Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                   // double tX = trans.get(0);
                  //  double tY = trans.get(1);
                   // double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    //double rX = rot.firstAngle;
                    //double rY = rot.secondAngle;
                    //double rZ = rot.thirdAngle;
               // }
        //    }
        //    else {
        //        telemetry.addData("VuMark", "not visible");
        //    }
            /* --------------end of pictograph reading, vuMark is the data to pass down */
            ///
            ///
            // control the servo to move the beam to the middle of the two jewels

            // Display the current value
            
            
            servoRight.setPosition(0.35);
            sleep(1000);
            telemetry.addData("Servo Position", "%5.2f", servoRight.getPosition());
            

            // after the beam is at the right place, read the jewel color information feedback
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (jewelSensorColorRight.red() * SCALE_FACTOR),
                    (int) (jewelSensorColorRight.green() * SCALE_FACTOR),
                    (int) (jewelSensorColorRight.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();
           // sleep(5000);
            

            if (hsvValues[0]>150 && hsvValues[0]<250 ) // this is the case for the jewel is blue color, then robot move forward.
            {
                DriveDistance(0.5,0.5,300);// talk to DC motor to move forward.
                
                forward = true;
            }
            else  // this is the case for the jewel is read color, then robot moves backward
            {
                DriveDistance(0.5,0.5,-300);
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
           
            DriveDistance(0.5,0.5,3000);
            sleep(2000);
            DriveDistance(1,0,1500);
            sleep(2000);
            servoArm.setPosition(0.58);
            DriveDistance(0,1,-1500);
            sleep(2000);
            
            // add more program to move more clyph if there is enough time.
            telemetry.addData("time used so far ", runtime.time());
            telemetry.update();
            if (runtime.time()<4)
            {
                // go to the safe zone.

            }
            telemetry.update();
            //servoArm.setPosition(0.58);
             //DriveDistance(0.5,0.5,-3000);
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
       // reset motor encoder and using encoder
       leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // going forward distance is positive, otherwise negative.
       leftMotor.setTargetPosition(-distance);
       rightMotor.setTargetPosition(distance);
       
       leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       
       DriveFoward(powerLeft, powerRight);
       // keep looping until motor reach to the target position
       while(leftMotor.isBusy() && rightMotor.isBusy())
       {
       }
       StopDriving();
       leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
}
