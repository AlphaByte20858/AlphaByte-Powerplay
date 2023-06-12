package org.firstinspires.ftc.teamcode.bytetonomo;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Autonomoteste_odometria extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0,0, Math.toRadians(180));

        Trajectory j1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(5,2), Math.toRadians(90))
                .build();

        drive.followTrajectory(j1);

    }
}