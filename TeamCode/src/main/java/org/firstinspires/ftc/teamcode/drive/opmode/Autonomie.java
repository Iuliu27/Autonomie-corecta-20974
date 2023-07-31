package org.firstinspires.ftc.teamcode.drive.opmode.OpenCV;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;


import java.util.ArrayList;
@Config
@Autonomous(name = "autonomiestanga")
public class   Autonomie extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        if (opModeIsActive()) {
            waitForStart();
            TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-37.46, -70.11, Math.toRadians(90.00)))
                    .forward(10)
                    .build();
            drive.setPoseEstimate(untitled0.start());

            while (opModeIsActive() && !isStopRequested()) {
                drive.followTrajectorySequence(untitled0);
                break;

            }
        }


    }


}