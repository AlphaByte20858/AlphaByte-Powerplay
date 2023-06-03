package org.firstinspires.ftc.teamcode.Testes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Encoder2", group="OpMode")
public class Linear2 extends OpMode{
    public DcMotorEx Arm;


    public static PIDCoefficients pidCoeffsa = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsa = new PIDCoefficients(0, 0, 0);
    double integrala = 0;
    double errora = 0;
    double currvela = 0;
    double derivatea = 0;
    double deltaErrora = 0;
    ElapsedTime tempo = new ElapsedTime();

    private double lastErrora = 0;

    @Override
    public void init() {

//        Arm.setTargetPosition(); botar a posição
        Arm = hardwareMap.get(DcMotorEx.class, "Arm");
        Arm.setDirection(DcMotor.Direction.FORWARD);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    public void loop() {
        boolean poderCima = gamepad2.right_bumper;
        boolean poderBaixo = gamepad2.left_bumper;
        boolean poderImovel = gamepad2.x;
        double pow;

        if (poderCima) {
            pow = pidLinear(-0.9);
            Arm.setPower(pow);
        }

        if (poderBaixo) {
            pow = pidLinear(0.5);
            Arm.setPower(pow);
        }
        if (poderImovel) {
            pow =  pidLinear(-0.1);
            Arm.setPower(pow);
        }
        else {
            pow = 0;
            Arm.setPower(pow);

        }
        telemetry.addData("A potencia do motor do sistema linear é de", pow);
        telemetry.addData("Posição do enconder", Arm.getCurrentPosition());

    }
    public double pidLinear(double velocidade) {


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

}
