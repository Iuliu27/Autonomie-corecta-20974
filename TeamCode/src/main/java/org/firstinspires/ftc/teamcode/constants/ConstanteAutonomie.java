package org.firstinspires.ftc.teamcode.constants;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;

public class ConstanteAutonomie {

public static SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

 public static Vector2d MyCordonates = new Vector2d(23 , 34);

 public static Trajectory Da = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
         .lineToConstantHeading(MyCordonates)
         .build();

}

