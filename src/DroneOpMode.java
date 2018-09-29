package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.lang.annotation.Target;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class DroneOpMode extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor armMotor;

    private Servo leftArmServo;
    private Servo rightArmServo;

    public static final double LEFTARMSERVO_CLOSED = .98;
    public static final double LEFTARMSERVO_OPEN = .5;
    public static final double RIGHTARMSERVO_CLOSED = 0;
    public static final double RIGHTARMSERVO_OPEN = .45;

    // Define class members
    protected Servo   servoRight;
    protected Servo   servoLeft;
    
    // todo: write your code here
    
    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
       
        servoRight = hardwareMap.get(Servo.class, "rightServo");
        servoLeft = hardwareMap.get(Servo.class, "leftServo");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // wait for diver to press play
        waitForStart();
        
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
           
        leftArmServo.setPosition(LEFTARMSERVO_CLOSED);
        rightArmServo.setPosition(RIGHTARMSERVO_CLOSED);
 
        double motorPower = 0;
        double steering = 0;
       
        double leftPower = 0;
        double rightPower = 0;
        
        double armPower = 0;
        double leftArmTgtServo = LEFTARMSERVO_OPEN;
        double rightArmTgtServo = RIGHTARMSERVO_OPEN;
        
        double armMotorPosition = armMotor.getCurrentPosition();
        
        int loopCount = 0;
        boolean slowMode = false;
        boolean parkMode = false;

        servoRight.setPosition(0.95);
        telemetry.addData("ServoRight is ", servoRight.getPosition());
        
        servoLeft.setPosition(0.0);
        telemetry.addData("ServoLeft is ", servoLeft.getPosition());
        
        while (opModeIsActive())
        {
            // make sure color sensor servos not open
            servoRight.setPosition(0.95);
        
            servoLeft.setPosition(0);
            loopCount++;
            
            // right stick is throttle
            motorPower = this.gamepad1.right_stick_y;
            
            // right stick steering
            steering = this.gamepad1.right_stick_x;
            
            // if the a button is pressed then toggle if you are in slowMode
            if (this.gamepad1.a) 
            {
                slowMode = !slowMode;
            }
            
            if (slowMode) 
            {
                motorPower = motorPower/4.0;
                steering = steering/4.0;
            }
            
            leftPower = motorPower - steering;
            rightPower = -(motorPower + steering);
            
            // Move arm up and down
            if (this.gamepad1.dpad_up)
            {
                armPower = -0.5;  
                telemetry.addData("DPAD: ", "UP");
                
                // if not driving
                if (leftPower == 0 && rightPower == 0)
                {
                    // move forward
                    leftPower = 0.15;
                    rightPower = -0.15;
                }
            }
            else if (this.gamepad1.dpad_down)
            {
                armPower = 0.5;
                telemetry.addData("DPAD: ", "DOWN");
                
                // if not driving
                if (leftPower == 0 && rightPower == 0)
                {
                    // move backwards
                    leftPower = -0.15;
                    rightPower = 0.15;
                }
 
            }
            else
            {
                armPower =0.0;
            }
            
            leftPower = Range.clip(leftPower, -1, 1);
            leftMotor.setPower(leftPower);
            rightPower = Range.clip(rightPower, -1, 1);
            rightMotor.setPower(rightPower);
            
            armMotor.setPower(armPower);
            
            int position = leftMotor.getCurrentPosition();
            double power = leftMotor.getPower();
            
            // OPERATOR
            if (this.gamepad1.left_bumper) 
            {
                rightArmTgtServo = RIGHTARMSERVO_OPEN;
                leftArmTgtServo = LEFTARMSERVO_OPEN;
           }
            if (this.gamepad1.right_bumper)
            {
                rightArmTgtServo = RIGHTARMSERVO_CLOSED;
                leftArmTgtServo = LEFTARMSERVO_CLOSED;
            }
            
            if (this.gamepad1.left_trigger > 0)
            {
                rightArmTgtServo -= 0.01;
                leftArmTgtServo += 0.01;
            }
            if (this.gamepad1.right_trigger > 0)
            {
                rightArmTgtServo += 0.01;
                leftArmTgtServo -= 0.01;
            }

            leftArmTgtServo = Range.clip(leftArmTgtServo, LEFTARMSERVO_OPEN, LEFTARMSERVO_CLOSED);           
            leftArmServo.setPosition(leftArmTgtServo);
            rightArmTgtServo = Range.clip(rightArmTgtServo, RIGHTARMSERVO_CLOSED, RIGHTARMSERVO_OPEN);           
            rightArmServo.setPosition(rightArmTgtServo);
            
            armMotorPosition = armMotor.getCurrentPosition();
            
            telemetry.addData("Target Power", leftPower);
            telemetry.addData("Motor Power", power);
            telemetry.addData("Motor Position", position);
            telemetry.addData("Left Arm Servo Position", leftArmServo.getPosition());
            telemetry.addData("Right Arm Servo Position", rightArmServo.getPosition());
            telemetry.addData("Arm Motor Position", armMotorPosition);
            telemetry.addData("Loop Count", loopCount);
            telemetry.addData("Slow Mode", slowMode);
            telemetry.update();
            
            if (this.gamepad1.y)
            {
                parkRobot();
            }
        }
    }
    
    protected void parkRobot()
    {
        // drop the arm
        armMotor.setPower(0.5);
        sleep(100);
        
        // drive forward
        leftMotor.setPower(-0.8);
        rightMotor.setPower(0.8);
        sleep(100);

        armMotor.setPower(0.0);
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }
}
