package org.firstinspires.ftc.teamcode.Testes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous(name="TesteOdometria", group="LinearOpMode")
public class TesteOdometria extends LinearOpMode{

    private Encoder leftEncoder, rightEncoder;
    public void runOpMode(){



        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        rightEncoder.setDirection(Encoder.Direction.REVERSE);


        rightEncoder.setDirection(Encoder.Direction.REVERSE);

        double a = 1;
        while(a == 1){
            telemetry.addData("Valora roda esquerda:",leftEncoder.getCurrentPosition());
            telemetry.addData("Valora roda direita:",rightEncoder.getCurrentPosition());
            telemetry.update();
    }

    }

}
