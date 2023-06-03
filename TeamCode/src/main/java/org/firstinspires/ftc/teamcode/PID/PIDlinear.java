package org.firstinspires.ftc.teamcode.PID;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="T", group="OpMode")
public class PIDlinear extends OpMode {


    DcMotorEx Arm;
    public Servo servoMotor = null;
    double powServo = 0;

    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0);
    double integral = 0;
    double error = 0;
    double currvel = 0;
    double derivate = 0;
    double deltaError = 0;


    ElapsedTime tempo = new ElapsedTime();

    private double lastError = 0;

    @Override
    public void init() {
        Arm = hardwareMap.get(DcMotorEx.class, "Arm");

        Arm.setDirection(DcMotor.Direction.REVERSE);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servoMotor = hardwareMap.get(Servo.class, "Servo");

    }

    public void loop() {
       linear();
    }

    public void linear(){
        if (gamepad1.right_bumper) {
            Arm.setPower(pidLinear(0.6));
        } else if (gamepad1.left_bumper) {
            Arm.setPower((pidLinear(-0.6)));
        }
        if (gamepad1.x) {
            Arm.setPower((pidLinear(0.6)));
            Arm.setPower((pidLinear(-0.6)));
        }
        if (gamepad1.dpad_down) {
            Arm.setTargetPosition(1000);
            pidLinear(9);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else {
            Arm.setPower(0);

        }
        telemetry.addData("O erro é de", error);
        telemetry.addData("O curvel é de", currvel);
        telemetry.addData("O deltaerro é de", deltaError);
        telemetry.addData("O ULTIMO erro é de", lastError);
    }

    public void servo() {

        // Criação das varíaveis tanto de força quanto dos botões
        // utilizados para abrir e fechar o servo

        boolean poderAberto = gamepad2.a;
        boolean poderFechado = gamepad2.b;
        double aberto = 0;
        double fechado = 1;

        // Função que define o servo estar aberto

        if (poderAberto) {
            powServo = aberto;
            servoMotor.setPosition(powServo);
        }

        // Função que define o servo estar fechado

        else if (poderFechado) {
            powServo = fechado;
            servoMotor.setPosition(powServo);
        }

        // Função que mostra o poder do servo no Drive Hub
        telemetry.addData("A potencia do motor do servo é de:", powServo);
    }

    public double pidLinear(double velocidade) {


        currvel = Arm.getVelocity();

        error = velocidade - currvel;

        integral += error * tempo.seconds();

        deltaError = (error - lastError);

        derivate = deltaError / tempo.seconds();

        lastError = error;

        pidGains.p = error * pidCoeffs.p;
        pidGains.i = integral * pidCoeffs.i;
        pidGains.d = derivate * pidCoeffs.d;

        tempo.reset();

        double output = (pidGains.p + pidGains.i + pidGains.d + velocidade);
        return output;


    }
}

