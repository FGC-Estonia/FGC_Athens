package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BallPusher {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final boolean protect;

    private ElapsedTime runtime = new ElapsedTime();

    private Servo leftPusher;
    private Servo rightPusher;
    private DcMotorEx hand;


    public BallPusher(boolean protect, HardwareMap hardwareMap, Telemetry telemetry){
        this.protect = protect;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        mapMotors();
    }

    private void mapMotors(){
        try {
            runtime.reset();
            hand = hardwareMap.get(DcMotorEx.class, "Motor_Port_3_EH");
            hand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hand.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            hand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } catch (Exception e){
            telemetry.addData("Hand error", e.getMessage());
        }
        /*
        leftPusher = hardwareMap.get(Servo.class, "Servo_Port_0_CH");
        rightPusher = hardwareMap.get(Servo.class, "Servo_Port_1_CH");

         */
    }

    public void moveHands(boolean leftState, boolean rightState){
        //hand.setPower(1);
        telemetry.addData("hand position", hand.getCurrentPosition());
        telemetry.addData("time", runtime.milliseconds());
        telemetry.addData("rightstate", rightState);
        if (rightState){
            hand.setTargetPosition(145);
            hand.setPower(1);
        } else {
            hand.setPower(1);
            hand.setTargetPosition(0);
        }
        if (hand.getCurrentPosition()<5){
            hand.setPower(0);
        }
        /*
        double leftPosition;
        double rightPosition;

        if (leftState){
            leftPosition = 0;
        } else {
            leftPosition  = 1;
        }
        if (rightState){
            rightPosition = 1;
        } else {
            rightPosition = 0;
        }

        leftPusher.setPosition(leftPosition);
        rightPusher.setPosition(rightPosition);

         */
    }
}
