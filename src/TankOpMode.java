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

public class TankOpMode extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor armMotor;

    private Servo armServo;
    private DistanceSensor leftColor;

    public static final double ARMSERVO_CLOSED = .55;
    public static final double ARMSERVO_OPEN = .75;

    // todo: write your code here
    
    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");

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
           
        armServo.setPosition(ARMSERVO_OPEN);
        
        double leftPower = 0;
        double rightPower = 0;
        double armPower = 0;
        double armTgtServo = ARMSERVO_OPEN;
        
        int loopCount = 0;
        
        while (opModeIsActive())
        {
            loopCount++;
            
            leftPower = this.gamepad1.left_stick_y;
            rightPower = -this.gamepad1.right_stick_y;
            if (this.gamepad1.dpad_up)
            {
                armPower = -0.5;  
                telemetry.addData("DPAD: ", "UP");
            }
            else if (this.gamepad1.dpad_down)
            {
                armPower = 0.5;
                telemetry.addData("DPAD: ", "DOWN");
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
            
            if (this.gamepad1.a) 
            {
                armTgtServo = ARMSERVO_OPEN;
            }
            if (this.gamepad1.b) 
            {
                armTgtServo = ARMSERVO_CLOSED;
            }
            
            if (this.gamepad1.left_bumper) 
            {
                armTgtServo = ARMSERVO_OPEN;
            }
            if (this.gamepad1.right_trigger >0)
            {
                armTgtServo = ARMSERVO_CLOSED;
            }
            if (this.gamepad1.left_trigger > 0)
            {
                armTgtServo += 0.05;
            }
            if (this.gamepad1.right_trigger > 0)
            {
                armTgtServo -= 0.05;
            }

            armTgtServo = Range.clip(armTgtServo, ARMSERVO_CLOSED, ARMSERVO_OPEN);           
            armServo.setPosition(armTgtServo);
            
            telemetry.addData("Target Power", leftPower);
            telemetry.addData("Motor Power", power);
            telemetry.addData("Motor Position", position);
            telemetry.addData("Arm Servo Position", armServo.getPosition());
            telemetry.addData("Arm Servo Target", armTgtServo);
            telemetry.addData("Loop Count", loopCount);
            telemetry.update();
        }
    }
}
