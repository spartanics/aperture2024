package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class samHWmarkers extends LinearOpMode {
    Servo testServo;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        testServo = hardwareMap.get(Servo.class, "test");
        waitForStart();

        if (isStopRequested()) return;
        TrajectorySequence trajseq = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(10)
                .strafeRight(5)
                .splineTo(new Vector2d(30, 30), Math.toRadians(0))
                .strafeLeft(10)
                .back(3)
                .splineTo(new Vector2d(0,0),Math.toRadians(0))
                .build();

        Trajectory displacetraj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    testServo.setPosition(0.5);
                })
                .splineTo(new Vector2d(30, 0), Math.toRadians(180))
                .build();

        Trajectory temporaltraj = drive.trajectoryBuilder(displacetraj.end(), Math.toRadians(0))
                .splineTo(new Vector2d(30, 30), Math.toRadians(180))
                //example delay (in theory)
                .addTemporalMarker (2, () -> {
                testServo.setPosition(0.5000001);
                })
                .addTemporalMarker (2.5, () -> {
                testServo.setPosition(0.5);
                })
                .splineTo(new Vector2d(0,0), Math.toRadians(0))
                .build();

        Trajectory spatialtraj = drive.trajectoryBuilder(temporaltraj.end(), Math.toRadians(0))
                .splineTo(new Vector2d(5, -15), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(3, -4), () -> {
                testServo.setPosition(1);
                })
                .build();

        testServo.setPosition(0);
//        drive.followTrajectory(displacetraj);
//
//        drive.followTrajectory(temporaltraj);
//
//        drive.followTrajectory(spatialtraj);

        drive.followTrajectorySequence(trajseq);


    }
}
