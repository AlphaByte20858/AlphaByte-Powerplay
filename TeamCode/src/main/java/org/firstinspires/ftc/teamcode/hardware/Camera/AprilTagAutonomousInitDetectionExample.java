
/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.hardware.Camera;
// importação das bibliotecas utilizadas
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.Testes.TeleOpTeste;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Autonomous

public class AprilTagAutonomousInitDetectionExample extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public TeleOpTeste Movi = new TeleOpTeste();

    static final double FEET_PER_METER = 3.28084;

    // Intrínsecos da lente
    // AS UNIDADES SÃO PIXELS
    // NOTA: esta calibração é para a webcam C920 em 800x448.
    // Você precisará fazer sua própria calibração para outras configurações!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;


    // UNIDADES SÃO METROS
    double tagsize = 2.000;

    // tags utilizadas
    int LEFT = 5;
    int MIDDLE = 6;
    int RIGHT = 7;

    // detecçao da tag nula
    AprilTagDetection tagOfInterest = null;
    //motores desligados
    public DcMotor motorEsquerdoF, motorEsquerdoT, motorDireitoF, motorDireitoT = null;


    int curArm;
    DcMotorEx Arm;
    public Servo servoMotor;
    //coeficiente do pid
    public static PIDCoefficients pidCoeffsa = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsa = new PIDCoefficients(0, 0, 0);
    //variaveis criadas
    double integrala = 0;
    double errora = 0;
    double currvela = 0;
    double derivatea = 0;
    double deltaErrora = 0;
    ElapsedTime tempo = new ElapsedTime();

    private double lastErrora = 0;

    @Override
    public void runOpMode()
    {
        motorEsquerdoF = hardwareMap.get(DcMotor.class, "LeftDriveUp");
        motorDireitoF = hardwareMap.get(DcMotor.class, "RightDriveUp");
        motorEsquerdoT = hardwareMap.get(DcMotor.class, "LeftDriveDown");
        motorDireitoT = hardwareMap.get(DcMotor.class, "RightDriveDown");

        motorEsquerdoF.setDirection(DcMotor.Direction.REVERSE);
        motorDireitoF.setDirection(DcMotor.Direction.FORWARD);
        motorEsquerdoT.setDirection(DcMotor.Direction.REVERSE);
        motorDireitoT.setDirection(DcMotor.Direction.FORWARD);

        Arm = hardwareMap.get(DcMotorEx.class, "Arm");

        Arm.setDirection(DcMotorEx.Direction.REVERSE);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servoMotor = hardwareMap.get(Servo.class, "Servo");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * iniciar o looping da programação
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    // igualar o valor das tags
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    // avisar que a programação esta pronta
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    //avisará se a tag nao for vista
                    telemetry.addLine("Não vejo a TAG :(");

                    if(tagOfInterest == null)
                    {
                        //avisará se a tag nao existir
                        telemetry.addLine("(A TAG nunca foi vista)");
                    }
                    else
                    {
                        //avisará qual foi a ultima tag detectada
                        telemetry.addLine("\nMas já vimos a TAG anteriormente. Última vez visto em:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("nao foi  :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(A tag foi vista");
                }
                else
                {
                    telemetry.addLine("\nMas já vimos a TAG anteriormente. Última vez visto em:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update da telimetria */
        if(tagOfInterest != null)
        {
            //detectar tag e avisar qual foi detectada
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            //avisará caso a tag não for detectada
            telemetry.addLine("a tag nao foi vista :(");
            telemetry.update();
        }

        servoMotor.setPosition(0);
        sleep(1500);

        curArm = 400;
        Arm.setTargetPosition(curArm);
        Arm.setPower(pidLinear(0.7));
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        sleep(700);
        allMotorsPower(-0.75,0.75,0.75,-0.75);

        /* Actually do something useful */
        if(tagOfInterest == null){
            //tragetoria escolhida
        }else if(tagOfInterest.id == LEFT){
// tragetoria esquerda
            sleep(800);
            allMotorsPower(0,0,0,0);
            sleep(600);
            allMotorsPower(-0.65,-0.65,-0.65,-0.65);
            sleep(400);
            allMotorsPower(0,0,0,0);


//tragetoria para meio
        }else if(tagOfInterest.id == MIDDLE){

            sleep(800);
            allMotorsPower(0,0,0,0);
            sleep(600);
            allMotorsPower(0,0,0,0);

        }else{
            //tragetoria direita

            sleep(800);
            allMotorsPower(0,0,0,0);
            sleep(600);
            allMotorsPower(0.65,0.65,0.65,0.65);
            sleep(400);
            allMotorsPower(0,0,0,0);

        }
        sleep(500);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setPower(pidLinear(0));


    }
    //calculo da telimetria para a camera
    public void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    //variaveis dos motores
    public void allMotorsPower ( double paMEF, double paMDF, double paMET, double paMDT){
        motorEsquerdoF.setPower(paMEF);
        motorDireitoF.setPower(paMDF);
        motorEsquerdoT.setPower(paMET);
        motorDireitoT.setPower(paMDT);
    }
    public double pidLinear(double velocidade) {
// calculo de pid para locomoção

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