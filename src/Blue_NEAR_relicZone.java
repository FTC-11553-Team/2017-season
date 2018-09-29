/* Copyright (c) 2017 FIRST. All rights reserved.
 */
package org.firstinspires.ftc.teamcode;
// Servo

import android.app.Activity;
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

@Autonomous(name="Blue_NEAR_relicZone", group ="Blue")
//@Disabled
public class Blue_NEAR_relicZone extends AutonomyOpMode 
{
    
    @Override public void knockTheJewel()
    {
        //control left and right arm servos to clench the glyph (If you clench them, the servo is out 18*18*18 cube)
        leftArmServo.setPosition(0.97);
        rightArmServo.setPosition(0);
        // control the servo to move the beam to the middle of the two jewels
        servoLeft.setPosition(0.52);
        sleep(1000);
        forward = false;

        // Display the current value
        telemetry.addData("Servo Position", "%5.2f", servoLeft.getPosition());
            
        if (this.isBlueJewel(jewelSensorColorLeft))
        {
            // move forward
            DriveDistance(0.5,0.5,300);
            forward = true;
        }
        else
        {
            // move backward.
            DriveDistance(0.3,0,-100);
            
        }
            
        // move the beam back to original
        servoLeft.setPosition(0);
            
        if (!forward)
        {
            DriveDistance(0.3,0,100);       
        }
    }
    
    @Override public void putGlyphInCryptoBox()
    {
        raiseArm();
        DriveDistance(.5,.5,1000);
        leftArmServo.setPosition(0.97);
        rightArmServo.setPosition(0);
        tankTurn(1,1500);
        dropArm();
        leftArmServo.setPosition(0.55);
        rightArmServo.setPosition(0.5);
        DriveDistance(.5,.5,500);
        DriveDistance(0.5,0.5,-250);
      //  DriveDistance(1,1,700);
       // leftArmServo.setPosition(0.5);
       // rightArmServo.setPosition(0.5);
        
    }
    
        
    @Override public void park()
    {
    }
}
