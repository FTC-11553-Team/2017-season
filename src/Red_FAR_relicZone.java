package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.AutonomyOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red_FAR_relicZone", group ="Red")
public class Red_FAR_relicZone extends AutonomyOpMode 
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
            // move backward
            DriveDistance(0,0.3,-120);
            
        }
        else
        {
            // move foward
            DriveDistance(0.5,0.5,120);
            forward = true;
        }
            
        // move the beam back to original
        servoRight.setPosition(0.95);
            
        if (!forward)
        {
            DriveDistance(0,0.3,120);       
        }
    }
    
    @Override public void putGlyphInCryptoBox()
    {
        
        raiseArm();
        leftArmServo.setPosition(0.98);
        rightArmServo.setPosition(0);
        DriveDistance(.305,0.6,1800);
        //DriveDistance(1,1,300);
        dropArm();
        leftArmServo.setPosition(0.45);
        rightArmServo.setPosition(0.5);
        //sleep(1000);
        DriveDistance(1,1,500);
        DriveDistance(0.5,0.5,-300);
    }
        
    @Override public void park()
    {
    }
}
