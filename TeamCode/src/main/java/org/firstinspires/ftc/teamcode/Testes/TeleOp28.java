package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PID.PIDlinear;
import org.firstinspires.ftc.teamcode.PID.PIDmovi;


@TeleOp(name="Teste28", group="OpMode")
public class TeleOp28 extends OpMode {


    PIDmovi pidm = new PIDmovi();


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


    // criação dos mostores
    DcMotorEx MDF,MEF,MDT,MET = null;


    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffs1 = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains1 = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffs2 = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains2 = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffs3 = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains3 = new PIDCoefficients(0, 0, 0);


    double integral = 0;
    double error = 0;
    double currvel = 0;
    double derivate = 0;
    double deltaError = 0;
    private double lastError = 0;

    double integral1 = 0;
    double error1 = 0;
    double currvel1 = 0;
    double derivate1 = 0;
    double deltaError1 = 0;
    private double lastError1 = 0;

    double integral2 = 0;
    double error2 = 0;
    double currvel2 = 0;
    double derivate2 = 0;
    double deltaError2 = 0;
    private double lastError2 = 0;

    double integral3 = 0;
    double error3 = 0;
    double currvel3 = 0;
    double derivate3 = 0;
    double deltaError3 = 0;
    private double lastError3 = 0;


    ElapsedTime tempo = new ElapsedTime();

    private double lastErrora = 0;

    @Override
    public void init() {
        Arm = hardwareMap.get(DcMotorEx.class, "Arm");

        Arm.setDirection(DcMotorEx.Direction.REVERSE);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servoMotor = hardwareMap.get(Servo.class, "Servo");


        MEF  = hardwareMap.get(DcMotorEx.class, "LeftDriveUp");
        MDF  = hardwareMap.get(DcMotorEx.class, "RightDriveUp");
        MET = hardwareMap.get(DcMotorEx.class, "LeftDriveDown");
        MDT = hardwareMap.get(DcMotorEx.class, "RightDriveDown");

        modemoto(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        modemoto(DcMotor.RunMode.RUN_USING_ENCODER);

        MEF.setDirection(DcMotor.Direction.REVERSE);
        MDF.setDirection(DcMotor.Direction.FORWARD);//REVERSE
        MET.setDirection(DcMotor.Direction.REVERSE);
        MDT.setDirection(DcMotor.Direction.FORWARD);//FORWARD



    }

    public void loop() {
        linear();
        servo();
        mover();
        telemetry.update();
    }

    public void mover(){

        // Criação das váriaveis de força para a movimentação
        double axial   = gamepad1.right_trigger - gamepad1.left_trigger;
        double lateral = gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // Criação de váriaveis para a movimentação em 8 direções
        double absaxial = Math.abs(axial);
        double abslateral = Math.abs(lateral);
        double absyaw= Math.abs(yaw);
        double denominador = Math.max(absaxial + abslateral + absyaw, 1);

        // Calculos para a movimentação das rodas omnidirectionais mecanum
        double motorEsquerdoFf = (axial + lateral + yaw / denominador);
        double motorDireitoFf = (axial - lateral - yaw / denominador);
        double motorEsquerdoTf = (axial - lateral + yaw / denominador);
        double motorDireitoTf = (axial + lateral - yaw / denominador);

        // Função que define que as Forças utilizadas em cada motor
        if(gamepad1.right_bumper){
            PIDVelMove(motorEsquerdoFf, motorDireitoFf , motorEsquerdoTf, motorDireitoTf  );
        }
        else {
            PIDVelMove(motorEsquerdoFf * 0.7, motorDireitoFf * 0.7, motorEsquerdoTf * 0.7, motorDireitoTf * 0.7);
        }

        // Nessa sequencia de if nos selecionamos as setas dos controles como meios
        // de movimentação mais precisa, sendo mais lentas e tendo um controle em ambas as 8 direções

        // Seta de baixo
        if(gamepad1.dpad_down){
            allMotorsPower(-0.5,-0.5,-0.5,-0.5);
        }

        // Seta de cima
        if(gamepad1.dpad_up){
            allMotorsPower(0.5,0.5,0.5,0.5);
        }

        // Seta da direita
        if(gamepad1.dpad_right){
            allMotorsPower(0.5,-0.5,-0.5,0.5);
        }

        // Seta da esquerda
        if(gamepad1.dpad_left){
            allMotorsPower(-0.5,0.5,0.5,-0.5);
        }




        // Nestas linhas de códigos nós usamos o comando Telemetry para
        // Conseguirmos ver as forças dos motores de movimentação no Drive Hub

        telemetry.addData("A potencia do motorEsquerdoF é de:", motorEsquerdoFf);
        telemetry.addData("A potencia do motorDireitoF é de:", motorDireitoFf);
        telemetry.addData("A potencia do motorEsquerdoT é de:", motorEsquerdoTf);
        telemetry.addData("A potencia do motorDireitoT é de:", motorDireitoTf);

    }
    public void linear(){
        if (gamepad2.right_bumper) {
            Arm.setPower(pidLinear(0.6));
        } else if (gamepad2.left_bumper) {
            Arm.setPower((pidLinear(-0.6)));
        }
        else if (gamepad2.x) {
            Arm.setPower((pidLinear(0.6)));
            Arm.setPower((pidLinear(-0.1)));
        }
        else {
            Arm.setPower(0);

        }
        telemetry.addData("Encoder teste", Arm.getCurrentPosition());
        if(gamepad2.dpad_down){
            telemetry.addData("Encoder junção terrena", Arm.getCurrentPosition());
        }
        if(gamepad2.dpad_right){
            telemetry.addData("Encoder junção baixa", Arm.getCurrentPosition());
        }
        if(gamepad2.dpad_left){
            telemetry.addData("Encoder alta", Arm.getCurrentPosition());
        }
        if(gamepad2.dpad_up){
            telemetry.addData("Encoder media", Arm.getCurrentPosition());
        }

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
    public double MDFpid(double velocidade) {


        currvel = MDF.getVelocity();

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
    public double MEFpid(double velocidade1) {


        currvel1 = MEF.getVelocity();

        error1 = velocidade1 - currvel1;

        integral1 += error1 * tempo.seconds();

        deltaError1 = (error1 - lastError1);

        derivate1 = deltaError1 / tempo.seconds();

        lastError1 = error1;

        pidGains1.p = error1 * pidCoeffs1.p;
        pidGains1.i = integral1 * pidCoeffs1.i;
        pidGains1.d = derivate1 * pidCoeffs1.d;
        tempo.reset();


        double output1 = (pidGains1.p + pidGains1.i + pidGains1.d + velocidade1);
        return output1;
    }
    public double METpid(double velocidade2) {


        currvel2 = MET.getVelocity();

        error2 = velocidade2 - currvel2;

        integral2 += error2 * tempo.seconds();

        deltaError2 = (error2 - lastError2);

        derivate2 = deltaError2 / tempo.seconds();

        lastError2 = error2;

        pidGains2.p = error * pidCoeffs2.p;
        pidGains2.i = integral * pidCoeffs2.i;
        pidGains2.d = derivate * pidCoeffs2.d;

        tempo.reset();

        double output2 = (pidGains2.p + pidGains2.i + pidGains2.d + velocidade2);
        return output2;
    }
    public double MDTpid(double velocidade3) {


        currvel3 = MDT.getVelocity();

        error3 = velocidade3 - currvel3;

        integral3 += error3 * tempo.seconds();

        deltaError3 = (error3 - lastError3);

        derivate3 = deltaError3 / tempo.seconds();

        lastError3 = error3;

        pidGains3.p = error3 * pidCoeffs3.p;
        pidGains3.i = integral3 * pidCoeffs3.i;
        pidGains3.d = derivate3 * pidCoeffs3.d;

        tempo.reset();

        double output3 = (pidGains3.p + pidGains3.i + pidGains3.d + velocidade3);
        return output3;
    }
    public void PIDVelMoveAll(double speed){
        allMotorsPower(MEFpid(speed), MDFpid(speed),METpid(speed), MDTpid(speed));
    }
    public void PIDVelMove(double paMEF1, double paMDF1, double paMET1, double paMDT1 ){
        allMotorsPower(MEFpid(paMEF1),MDFpid(paMDF1), METpid(paMET1), MDTpid(paMDT1));
    }
    public void allMotorsPower(double paMEF, double paMDF, double paMET, double paMDT){
        MEF.setPower(paMEF);
        MDF.setPower(paMDF);
        MET.setPower(paMET);
        MDT.setPower(paMDT);
    }
    public void modemoto(DcMotor.RunMode mode){
        MDF.setMode(mode);
        MDT.setMode(mode);
        MEF.setMode(mode);
        MET.setMode(mode);
    }
}

