package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.AutonomyOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue_FAR_relicZone", group ="Blue")
public class Blue_FAR_relicZone extends AutonomyOpMode 
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
            DriveDistance(0.5,0.5,150);
            forward = true;
        }
        else
        {
            // move backward
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
        DriveDistance(1,.60,1750);
        //DriveDistance(1,1,200);
        dropArm();
        leftArmServo.setPosition(0.55);
        rightArmServo.setPosition(0.5);
        DriveDistance(0.8,1,400);
        DriveDistance(0.5,0.5,-300);
    }
        
    @Override public void park()
    {
    }
}
