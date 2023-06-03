/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.byteoperado;
// importação das bibliotecas utilizadas

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// definição da classe teleop, junto com
// o nome do arquivo e seu grupo
@TeleOp(name="Alpha control", group="OpMode")
public class TeleOpByte extends OpMode {

    // criação das variaveis de cada tipo
    // como dos motores e dos servos.
    DcMotorEx Arm;
    public Servo servoMotor = null;
    DcMotorEx MDF,MEF,MDT,MET = null;

    // criação dos arquivos para o uso do pid
    // de cada motor, tanto da movimentação e do sistema linear

    public static PIDCoefficients pidCoeffsa = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsa = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffs1 = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains1 = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffs2 = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains2 = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffs3 = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains3 = new PIDCoefficients(0, 0, 0);


    // definição do poder do servo
    double powServo = 0;

    // criação das variaveis de poder
    // pid do sistema linear
    double integrala = 0;
    double errora = 0;
    double currvela = 0;
    double derivatea = 0;
    double deltaErrora = 0;
    private double lastErrora = 0;

    // criação das variaveis de poder
    // pid do motor MEF
    double integral = 0;
    double error = 0;
    double currvel = 0;
    double derivate = 0;
    double deltaError = 0;
    private double lastError = 0;

    // criação das variaveis de poder
    // pid do motor MDF
    double integral1 = 0;
    double error1 = 0;
    double currvel1 = 0;
    double derivate1 = 0;
    double deltaError1 = 0;
    private double lastError1 = 0;

    // criação das variaveis de poder
    // pid do motor MET
    double integral2 = 0;
    double error2 = 0;
    double currvel2 = 0;
    double derivate2 = 0;
    double deltaError2 = 0;
    private double lastError2 = 0;

    // criação das variaveis de poder
    // pid do motor MDT
    double integral3 = 0;
    double error3 = 0;
    double currvel3 = 0;
    double derivate3 = 0;
    double deltaError3 = 0;
    private double lastError3 = 0;

    // criação da variavel de tempo
    ElapsedTime tempo = new ElapsedTime();




    int curArm;
    int MAX_HIGH = 3000;
    int MIN_HIGH = 0;

    @Override
    public void init() {

        // classificação do motor o sistema linear
        Arm = hardwareMap.get(DcMotorEx.class, "Arm");

        // definição da direção do sistema linear

        Arm.setDirection(DcMotorEx.Direction.REVERSE);

        // configurando dos encoders
        // dos motores do sistema linear

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // definição do motor do Servo

        servoMotor = hardwareMap.get(Servo.class, "Servo");

        // classificação dos motores de movimentação

        MEF  = hardwareMap.get(DcMotorEx.class, "LeftDriveUp");
        MDF  = hardwareMap.get(DcMotorEx.class, "RightDriveUp");
        MET = hardwareMap.get(DcMotorEx.class, "LeftDriveDown");
        MDT = hardwareMap.get(DcMotorEx.class, "RightDriveDown");

        // configurando os encoders da movimentação

        modemoto(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        modemoto(DcMotor.RunMode.RUN_USING_ENCODER);

        // definição das direções do robô

        MEF.setDirection(DcMotor.Direction.REVERSE);
        MDF.setDirection(DcMotor.Direction.FORWARD);
        MET.setDirection(DcMotor.Direction.REVERSE);
        MDT.setDirection(DcMotor.Direction.FORWARD);

        // codigo que impede do sistema linear
        // de descer

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // criação da função de loop
    public void loop() {
        // executamento das funções para o robô
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

        // Criação de váriaveis par8 a movimentação em 8 direções
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
            PIDVelMove(motorEsquerdoFf * 0.8, motorDireitoFf * 0.8, motorEsquerdoTf * 0.8, motorDireitoTf * 0.8);
        }

        // Nessa sequencia de if nos selecionamos as setas dos controles como meios
        // de movimentação mais precisa, sendo mais lentas e tendo um controle em ambas as 8 direções

        // Seta de baixo
        if(gamepad1.dpad_down){
            PIDVelMoveAll(-0.6);
        }

        // Seta de cima
        if(gamepad1.dpad_up){
            PIDVelMoveAll(0.6);
        }

        // Seta da direita
        if(gamepad1.dpad_right){
            PIDVelMove(0.6,-0.6,-0.6,0.6);
        }

        // Seta da esquerda
        if(gamepad1.dpad_left){
            PIDVelMove(-0.6,0.6,0.6,-0.6);
        }




        // Nestas linhas de códigos nós usamos o comando Telemetry para
        // Conseguirmos ver as forças dos motores de movimentação no Drive Hub

        telemetry.addData("A potencia do motorEsquerdoF é de:", motorEsquerdoFf);
        telemetry.addData("A potencia do motorDireitoF é de:", motorDireitoFf);
        telemetry.addData("A potencia do motorEsquerdoT é de:", motorEsquerdoTf);
        telemetry.addData("A potencia do motorDireitoT é de:", motorDireitoTf);

    }
    // Criação da função do sistema linear

    public void linear(){

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
        // aqui temos uma telemetria que mostra a posição
        // do encoder do motor do sistema linear
        telemetry.addData("Encoder teste", Arm.getCurrentPosition());

    }
    // criação da função do servo
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

    // esta é a parte do codigo em que criamos
    // nosso calculo pid do sistema linear
    public double pidLinear(double velocidade) {


        // calculo do pid

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
    // esta é a parte do codigo em que criamos
    // nosso calculo pid do motor da direta frente
    public double MDFpid(double velocidade) {

        // calculo do pid

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
    // esta é a parte do codigo em que criamos
    // nosso calculo pid do motor da esquerda frente
    public double MEFpid(double velocidade1) {

        // calculo do pid

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
    // esta é a parte do codigo em que criamos
    // nosso calculo pid do motor da esquerda trás
    public double METpid(double velocidade2) {

        // calculo do pid

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
    // esta é a parte do codigo em que criamos
    // nosso calculo pid do motor da direta trás
    public double MDTpid(double velocidade3) {


        // calculo do pid

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
    // Criação da função de movimentação com PID em que todos os motores
    // tem a mesma potencia
    public void PIDVelMoveAll(double speed){
        allMotorsPower(MEFpid(speed), MDFpid(speed),METpid(speed), MDTpid(speed));
    }
    // Criação da função de movimentação com PID em que cada motor
    // tem uma potencia diferente
    public void PIDVelMove(double paMEF1, double paMDF1, double paMET1, double paMDT1 ){
        allMotorsPower(MEFpid(paMEF1),MDFpid(paMDF1), METpid(paMET1), MDTpid(paMDT1));
    }
    // Criação da função de movimentação que define o poder de cada motor
    public void allMotorsPower(double paMEF, double paMDF, double paMET, double paMDT){
        MEF.setPower(paMEF);
        MDF.setPower(paMDF);
        MET.setPower(paMET);
        MDT.setPower(paMDT);
    }
    // Criação da função de identificação de modo de cada motor de moivmentação
    public void modemoto(DcMotor.RunMode mode){
        MDF.setMode(mode);
        MDT.setMode(mode);
        MEF.setMode(mode);
        MET.setMode(mode);
    }
}

