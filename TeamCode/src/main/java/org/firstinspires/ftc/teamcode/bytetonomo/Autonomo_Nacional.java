
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

package org.firstinspires.ftc.teamcode.bytetonomo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Camera.AprilTagDetectionPipeline;
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

@Autonomous(name="autoByteB", group="OpMode")

public class Autonomo_Nacional extends LinearOpMode

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


    int LEFT = 5;
    int MIDDLE = 6;
    int RIGHT = 7;

    AprilTagDetection tagOfInterest = null;

    public DcMotor motorEsquerdoF, motorEsquerdoT, motorDireitoF, motorDireitoT = null;

    int curArm;
    DcMotorEx Arm;
    public Servo servoMotor;

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

        motorEsquerdoF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDireitoF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorEsquerdoT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDireitoT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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
         * The INIT-loop:
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
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Não vejo a TAG :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(A TAG nunca foi vista)");
                    }
                    else
                    {
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

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("a tag nao foi vista :(");
            telemetry.update();
        }
        ///////

        int powerEst = 0;

        if(tagOfInterest == null){
            //default trajectory here if preferred
        }else if(tagOfInterest.id == LEFT){
            powerEst = -1000;

        }else if(tagOfInterest.id == MIDDLE){
            powerEst = 0;

        }else{
            //right trajectory
            powerEst = 1000;
        }

        servoMotor.setPosition(0);
        sleep(1500);

        curArm = 1400;
        Arm.setTargetPosition(curArm);
        Arm.setPower(pidLinear(0.7));
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);

        Movienco(0.8,-900, 900,900,-900);
        sleep(2000);
        Movienco(0.8,280, 280,280,280);
        sleep(2000);
        servoMotor.setPosition(1);
        sleep(1000);
        Movienco(0.8,-280, -280,-280,-280);
        sleep(1800);
        Movienco(0.8,-750, 750,750,-750);
        sleep(1800);
        Movienco(0.8,powerEst,powerEst,powerEst,powerEst);
        sleep(1800);
        Movienco(0,0,0,0,0);


        /* Actually do something useful */


    }

    public void Movienco(double pows, int posMEF, int posMDF ,int posMET, int posMDT){
        motorEsquerdoF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDireitoF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorEsquerdoT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDireitoT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorEsquerdoF.setTargetPosition(posMEF);
        motorDireitoF.setTargetPosition(posMDF);
        motorEsquerdoT.setTargetPosition(posMET);
        motorDireitoT.setTargetPosition(posMDT);

        motorEsquerdoF.setPower(pows);
        motorDireitoF.setPower(pows);
        motorEsquerdoT.setPower(pows);
        motorDireitoT.setPower(pows);

        motorEsquerdoF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDireitoF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorEsquerdoT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDireitoT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

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
    public void allMotorsPower ( double paMEF, double paMDF, double paMET, double paMDT){
        motorEsquerdoF.setPower(paMEF);
        motorDireitoF.setPower(paMDF);
        motorEsquerdoT.setPower(paMET);
        motorDireitoT.setPower(paMDT);
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