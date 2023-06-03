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


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Testeroda", group="Robot")


public class Testeroda extends OpMode {

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


    @Override
    public void init() {
        MEF  = hardwareMap.get(DcMotorEx .class, "LeftDriveUp");
        MDF  = hardwareMap.get(DcMotorEx.class, "RightDriveUp");
        MET = hardwareMap.get(DcMotorEx.class, "LeftDriveDown");
        MDT = hardwareMap.get(DcMotorEx.class, "RightDriveDown");

        MEF.setDirection(DcMotor.Direction.REVERSE);
        MDF.setDirection(DcMotor.Direction.FORWARD);//REVERSE
        MET.setDirection(DcMotor.Direction.REVERSE);
        MDT.setDirection(DcMotor.Direction.FORWARD);

        modemoto(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        modemoto(DcMotor.RunMode.RUN_USING_ENCODER);



    }
    public void loop(){
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
        if(gamepad1.a) {
            PIDVelMove(0, 0, 0, 1);
        }
        else if(gamepad1.b){
            PIDVelMove(0,0,1,0);
        }
        else if(gamepad1.x) {
            PIDVelMove(0, 1, 0, 0);
        }
        else if(gamepad1.y){
                PIDVelMove(1,0,0,0);
            }
        else{
            PIDVelMove(0,0,0,0);
        }
        telemetry.addLine("A = direita tras");
        telemetry.addLine("B = esquerda tras");
        telemetry.addLine("x = direita frente");
        telemetry.addLine("y = esquerda frente");



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

        double output = (pidGains1.p + pidGains1.i + pidGains1.d + velocidade1);
        return output;
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

        double output = (pidGains2.p + pidGains2.i + pidGains2.d + velocidade2);
        return output;
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

        double output = (pidGains3.p + pidGains3.i + pidGains3.d + velocidade3);
        return output;
    }
    public void PIDVelMoveAll(double speed){
        allMotorsPower(MEFpid(speed), MDFpid(speed), METpid(speed), MDTpid(speed));
    }
    public void PIDVelMove(double paMEF1, double paMDF1, double paMET1, double paMDT1 ){
        allMotorsPower(MEFpid(paMEF1), MDFpid(paMDF1), METpid(paMET1), MDTpid(paMDT1));
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