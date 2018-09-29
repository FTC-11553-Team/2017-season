package org.firstinspires.ftc.teamcode;

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

//@Autonomous(name="BaseAutonomy_relicZone", group ="ZZZ")
@Disabled
public class AutonomyOpMode extends LinearOpMode {

    //DcMotor makes it drive forward or backward
    protected DcMotor leftMotor;
    protected DcMotor rightMotor;
    protected DcMotor armMotor;

    protected static int TICK2INCHES = 120;

    // declare for Servo installed in the right
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   5;      // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    protected Servo   servoRight;
    protected Servo   servoLeft;
    protected Servo   leftArmServo;
    protected Servo   rightArmServo;
 
    protected boolean rampUp = true;
    protected boolean forward = false;

    // declare variables for jewel color sensor
    protected ColorSensor jewelSensorColorRight;
    protected ColorSensor jewelSensorColorLeft;

    protected double armMotorPosition = 0;

    // create a timer
    private ElapsedTime runtime = new ElapsedTime();
    
    @Override public void runOpMode() {

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        servoRight = hardwareMap.get(Servo.class, "rightServo");
        servoLeft = hardwareMap.get(Servo.class, "leftServo");
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");

        jewelSensorColorRight = hardwareMap.get(ColorSensor.class, "rightColorSensor");
        jewelSensorColorLeft = hardwareMap.get(ColorSensor.class, "leftColorSensor");
        
        // before waiting for init, make sure sensor arms are up and arm is tight

        servoRight.setPosition(0.95);
        telemetry.addData("ServoRight is ", servoRight.getPosition());
        
        servoLeft.setPosition(0.0);
        telemetry.addData("ServoLeft is ", servoLeft.getPosition());

      //  servoArm.setPosition(0.679);
        telemetry.addData("leftArmServo is ", leftArmServo.getPosition());
        
        telemetry.addData("rightArmServo is ", rightArmServo.getPosition());
        
        
        
        // assume starting position is down.
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorPosition = armMotor.getCurrentPosition();
        
        telemetry.addData("Arm Motor is ", armMotorPosition);
        
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int colorrelativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View colorrelativeLayout = ((Activity) hardwareMap.appContext).findViewById(colorrelativeLayoutId);

        /* before waiting for init, check the hardware information - Camera*/
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) 
        {
         //   raiseArm();
            
            armMotorPosition = armMotor.getCurrentPosition();
            telemetry.addData("Arm Motor is ", armMotorPosition);

            knockTheJewel();

            sleep(CYCLE_MS);
            
            putGlyphInCryptoBox();

            park();
            
            // add more program to move more glyph if there is enough time.
            telemetry.addData("time used so far ", runtime.time());
            telemetry.update();
            
            if (runtime.time()<4)
            {
                // go to the safe zone.
            }
            telemetry.update();
        }
    }
    
    public void knockTheJewel()
    {
    }
    
    public void putGlyphInCryptoBox()
    {
    }
    
    public void park()
    {
    }
    
    public void raiseArm()
    {
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        armMotor.setTargetPosition(-1000);
        armMotor.setPower(0.5);
        
        // keep looping until motor reach to the target position
        while(armMotor.isBusy())
        {
        }
        armMotor.setPower(0.0);
    }
    
    public void dropArm()
    {
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        armMotor.setTargetPosition(-250);
        armMotor.setPower(0.5);
        
        // keep looping until motor reach to the target position
        while(armMotor.isBusy())
        {
        }
        armMotor.setPower(0.0);
    }
    
    public boolean isBlueJewel(ColorSensor colorSensor)
    {
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // after the beam is at the right place, read the jewel color information feedback
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                       (int) (colorSensor.green() * SCALE_FACTOR),
                       (int) (colorSensor.blue() * SCALE_FACTOR),
                       hsvValues);
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.update();
      //sleep(20000);
            
        if (hsvValues[0]>150 && hsvValues[0]<250 ) // this is the case for the jewel is blue color
        {
           
            return true;
        }
        else  // this is the case for the jewel is red color
        {
            return false;
        }
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
    
    //         tankTurn(1, 1550);  // 90 degrees left
    public void tankTurn (double power, int distance)
    {
       // reset motor encoder and using encoder
       leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       
       // going forward distance is positive, otherwise negative.
       leftMotor.setTargetPosition(distance);
       rightMotor.setTargetPosition(distance);
       
       leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       
       DriveFoward(power, power);
       
       // keep looping until motor reach to the target position
       while(leftMotor.isBusy() && rightMotor.isBusy())
       {
       }
       
       StopDriving();
       leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
