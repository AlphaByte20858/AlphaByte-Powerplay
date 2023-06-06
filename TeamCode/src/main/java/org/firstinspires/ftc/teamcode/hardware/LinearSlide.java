package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LinearSlide extends LinearOpMode {

    DcMotorEx Arm;
    public Servo servoMotor = null;
    double powServo = 0;

    public static PIDCoefficients pidCoeffsa = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsa = new PIDCoefficients(0, 0, 0);

    double integrala = 0;
    double errora = 0;
    double currvela = 0;
    double derivatea = 0;
    double deltaErrora = 0;

    ElapsedTime tempo = new ElapsedTime();

    private double lastErrora = 0;

    int curArm;

    @Override
    public void runOpMode() {
        Arm = hardwareMap.get(DcMotorEx.class, "Arm");

        Arm.setDirection(DcMotorEx.Direction.REVERSE);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setLinearUp() {
            Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Arm.setPower(pidLinear(0.6));
        }
    public void setLinearDown(){
            Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Arm.setPower((pidLinear(-0.6)));
    }
    public void setLinearGroundJuction(){
        curArm = 300;
        Arm.setTargetPosition(curArm);
        Arm.setPower(pidLinear(0.6));
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setLinearLowJuction(){
        curArm = 300;
        Arm.setTargetPosition(curArm);
        Arm.setPower(pidLinear(0.6));
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setLinearMediumJuction(){
        curArm = 300;
        Arm.setTargetPosition(curArm);
        Arm.setPower(pidLinear(0.6));
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setLinearHighJuction(){
        curArm = 300;
        Arm.setTargetPosition(curArm);
        Arm.setPower(pidLinear(0.6));
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double pidLinear(double velocidade){


        currvela = Arm.getVelocity();

        errora = velocidade - currvela;

        integrala += errora * tempo.seconds();

        deltaErrora = (errora - lastErrora);

        derivatea = deltaErrora / tempo.seconds();

        lastErrora = errora;

        pidGainsa.p = errora * pidCoeffsa.p;
        pidGainsa.i = integrala * pidCoeffsa.i;
        pidGainsa.d = derivatea * pidCoeffsa.d;

        tempo.reset();

        double outputA = (pidGainsa.p + pidGainsa.i + pidGainsa.d + velocidade);
        return outputA;
    }
    public void getLinearPosition(){
        Arm.getTargetPosition();
    }
    public void getLinearPower(){
        Arm.getPower();
    }
}

