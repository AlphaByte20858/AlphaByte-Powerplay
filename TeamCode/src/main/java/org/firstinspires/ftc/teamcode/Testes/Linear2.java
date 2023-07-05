package org.firstinspires.ftc.teamcode.Testes;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

    int curArm;
    private double lastErrora = 0;

    private DistanceSensor sensorRange;

    boolean isUp;

    @Override
    public void init() {

//        Arm.setTargetPosition(); botar a posição
        Arm = hardwareMap.get(DcMotorEx.class, "Arm");
        Arm.setDirection(DcMotor.Direction.FORWARD);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sensorRange = hardwareMap.get(DistanceSensor.class, "DistanciaSensor");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;


    }


    public void loop() {

        // nesta sequencia de ifs nós primeiro fazemos
        // o sistema linear ir elevar e abaixar

        // primeiro a criação do codigo de elevação do sistema linear
        if (gamepad2.right_bumper) {
            Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Arm.setPower(pidLinear(0.6));

        }
        // neste codigo nos fazemos o sistema linear
        // se abaixar
        else if (gamepad2.left_bumper) {
            Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Arm.setPower((pidLinear(-0.6)));
        }

        // Agora nesta sequencia de ifs nós criamos
        // funções para automatizar as alturas do sistema linear

        // nesta parte ele vai para media junção
        else if (gamepad2.dpad_right) {
            curArm = 2200;
            Arm.setTargetPosition(curArm);
            Arm.setPower(pidLinear(0.6));
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        // nesta parte ele vai para junção alta
        else if (gamepad2.dpad_up) {
            curArm = 3100;
            Arm.setTargetPosition(curArm);
            Arm.setPower(pidLinear(0.6));
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        // nesta parte ele vai para a junção terrea
        else if (gamepad2.dpad_down) {
            curArm = 300;
            Arm.setTargetPosition(curArm);
            Arm.setPower(pidLinear(0.6));
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        // nesta parte ele vai para a junção baixa
        else if (gamepad2.dpad_left) {
            curArm = 1400;
            Arm.setTargetPosition(curArm);
            Arm.setPower(pidLinear(0.6));
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        // E por fim, nessa junção ele para de exercer qualquer
        // força quando nenhum botão for pressionado
        else {
            Arm.setPower(0);
        }
        if(Arm.getCurrentPosition() > 500){
            isUp = true;
        }
        if (isUp = true && sensorRange.getDistance(DistanceUnit.CM) < 6) {
            isUp = false;
            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        // aqui temos uma telemetria que mostra a posição
        // do encoder do motor do sistema linear
        telemetry.addData("Encoder teste", Arm.getCurrentPosition());

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
