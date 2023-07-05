package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


public class Claw extends LinearOpMode {

    public Servo servoMotor = null;
    double powServo = 0;
    double open = 1;
    double locked = 0;

    @Override
    public void runOpMode() {
        servoMotor = hardwareMap.get(Servo.class, "Servo");
    }
    public void setServoOpen() {
        powServo = open;
        servoMotor.setPosition(powServo);
    }
    public void setServoLocked(){
        powServo = locked;
        servoMotor.setPosition(powServo);
    }
    public void getServoPosition(){
        servoMotor.getPosition();
    }
    public void setServoPosition(double position){
        servoMotor.setPosition(position);
    }
}

