package org.firstinspires.ftc.teamcode.sensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareDevice;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "AutonomoTeste", group = "LinearOpMode")

public class Gyro extends LinearOpMode {

    sensor robot   = new sensor();
    ElapsedTime     runTime = new ElapsedTime();

    Orientation lastAngle = new Orientation();
    double currAngle;


    @Override
    public void runOpMode() {
        turnTo(90);
        sleep(500);
        turnTo(-90);
    }

    public void resetAngle(){
        lastAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES);
        currAngle = 0;
    }
    public double getAngle(){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastAngle.firstAngle;

        if( deltaAngle > 180){
            deltaAngle -= 360;
        }
        else if (deltaAngle <= -180 ){
            deltaAngle += 360;
        }


        currAngle += deltaAngle;
        lastAngle = orientation;
        telemetry.addData("O angulo atual é de", orientation.firstAngle);

        return currAngle;

    }

    public void turn (double graus){

        double error = graus;

        resetAngle();
        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorpower = (error < 0 ? -0.5 : 0.5);
            robot.allMotorsPower(-5, 5, 5, 5);
            error = graus - currAngle;
            telemetry.addData("Erro", error);
            telemetry.update();
        }
    }
    public void turnTo(double graus){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        double error = graus - orientation.firstAngle;

        if (error > 180){
            error -= 360;
        }else if (error < 180){
            error += 360;
        }
        turn(error);
    }

}
