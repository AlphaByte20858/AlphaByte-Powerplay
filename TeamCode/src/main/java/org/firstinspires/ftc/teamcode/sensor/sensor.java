package org.firstinspires.ftc.teamcode.sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AUTO.AprilTagAutonomousInitDetectionExample;


@Autonomous(name = "AutonomoTeste2", group = "LinearOpMode")

public class sensor extends LinearOpMode {
    public AprilTagAutonomousInitDetectionExample cam = new AprilTagAutonomousInitDetectionExample();
    public DcMotor motorEsquerdoF, motorEsquerdoT, motorDireitoF, motorDireitoT = null;
    BNO055IMU imu;


    @Override
    public void runOpMode() {
        motorEsquerdoF = hardwareMap.get(DcMotor.class, "LeftDriveUp");
        motorDireitoF = hardwareMap.get(DcMotor.class, "RightDriveUp");
        motorEsquerdoT = hardwareMap.get(DcMotor.class, "LeftDriveDown");
        motorDireitoT = hardwareMap.get(DcMotor.class, "RightDriveDown");


        motorEsquerdoF.setDirection(DcMotor.Direction.REVERSE);
        motorDireitoF.setDirection(DcMotor.Direction.FORWARD);
        motorEsquerdoT.setDirection(DcMotor.Direction.REVERSE);
        motorDireitoT.setDirection(DcMotor.Direction.FORWARD);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        resetRuntime();
        waitForStart();

        if (opModeIsActive()) {

            cam.runOpMode();


            while (opModeIsActive() && motorDireitoF.isBusy() && motorDireitoT.isBusy() && motorEsquerdoF.isBusy() && motorEsquerdoT.isBusy()) {
                idle();

            }
        }
    }
        public void allMotorsPower ( double paMEF, double paMDF, double paMET, double paMDT){
            motorEsquerdoF.setPower(paMEF);
            motorDireitoF.setPower(paMDF);
            motorEsquerdoT.setPower(paMET);
            motorDireitoT.setPower(paMDT);
        }
    }
