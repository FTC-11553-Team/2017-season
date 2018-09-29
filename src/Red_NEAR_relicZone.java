/* Copyright (c) 2017 FIRST. All rights reserved.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red_NEAR_relicZone", group ="Red")
public class Red_NEAR_relicZone extends AutonomyOpMode 
{
    
    @Override public void knockTheJewel()
    {
        //control left and right arm servos to clench the glyph (If you clench them, the servo is out 18*18*18 cube)
        leftArmServo.setPosition(0.97);
        rightArmServo.setPosition(0);
        // control the servo to move the beam to the middle of the two jewels
        servoRight.setPosition(0.35);
        sleep(1000);

        // Display the current value
        telemetry.addData("Servo Position", "%5.2f", servoRight.getPosition());
            
        forward = false;
        
        if (this.isBlueJewel(jewelSensorColorRight))
        {
            // move backward.
            DriveDistance(0,0.3,-100);
           
        }
        else
        {
            // move forward
            DriveDistance(0.5,0.5,300);
             forward = true;
        }
            
        // move the beam back to original
        servoRight.setPosition(0.95);
            
        if (!forward)
        {
            DriveDistance(0,0.3,100);       
        }
    }
    @Override public void putGlyphInCryptoBox()
    {
        raiseArm();
        DriveDistance(.5,.5,1000);   // 4850
        leftArmServo.setPosition(0.97);
        rightArmServo.setPosition(0);
        tankTurn(1,-1500);  //-2343
        dropArm();
        leftArmServo.setPosition(0.55);
        rightArmServo.setPosition(0.5);
        DriveDistance(0.5,0.5,500);
        DriveDistance(0.5,0.5,-300);
        //leftArmServo.setPosition(0.4);
       // rightArmServo.setPosition(0.6);
    }
}
