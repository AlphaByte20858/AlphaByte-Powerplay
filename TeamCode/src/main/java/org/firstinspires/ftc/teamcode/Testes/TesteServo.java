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

package org.firstinspires.ftc.teamcode.Testes;
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
@TeleOp(name="Teste Servo", group="OpMode")
public class TesteServo extends OpMode {

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
    ElapsedTime tempoServo = new ElapsedTime();



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



        // definição das direções do robô

        MEF.setDirection(DcMotor.Direction.REVERSE);
        MDF.setDirection(DcMotor.Direction.FORWARD);
        MET.setDirection(DcMotor.Direction.REVERSE);
        MDT.setDirection(DcMotor.Direction.FORWARD);

        // codigo que impede do sistema linear
        // de descer

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MDF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MEF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MET.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // criação da função de loop
    public void loop() {
        // executamento das funções para o robô
        servo();
        telemetry.update();
    }


    public void servo() {

        // Criação das varíaveis tanto de força quanto dos botões
        // utilizados para abrir e fechar o servo

        boolean bottonServo = gamepad2.a;
        double aberto = 0;
        double fechado = 0.70;

        // Função que define o servo estar aberto

        if (bottonServo && tempoServo.seconds() >= 0.3) {
            if (powServo == aberto){
                powServo = fechado;
                servoMotor.setPosition(powServo);
            }
            else if(powServo == fechado){
                powServo = aberto;
                servoMotor.setPosition(powServo);
            }
            tempoServo.reset();
        }

        // Função que define o servo estar fechado

        if(gamepad2.right_bumper){
            powServo += 0.1;
            if (powServo >= 1 ){
                powServo = 1;
            }
            servoMotor.setPosition(powServo);
        }

        if(gamepad2.left_bumper){
            powServo -= 0.1;
            if (powServo <= 0 ){
                powServo = 0;
            }
            servoMotor.setPosition(powServo);
        }

        // Função que mostra o poder do servo no Drive Hub
        telemetry.addData("A potencia do motor do servo é de:", powServo);
    }

}

