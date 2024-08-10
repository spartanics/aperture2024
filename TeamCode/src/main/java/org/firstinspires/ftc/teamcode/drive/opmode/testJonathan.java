package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class testJonathan extends LinearOpMode {
    @Override



    public void runOpMode() throws InterruptedException {
        Servo testServo = hardwareMap.get(Servo .class, "test");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory ford = drive.trajectoryBuilder(new Pose2d())
                .addTemporalMarker(0.5, () -> {
                    testServo.setPosition(1);
                })
                .addTemporalMarker(1, () -> {
                    testServo.setPosition(0);
                })
                .addDisplacementMarker(30, () -> {
                    testServo.setPosition(1);
                })
                .splineToSplineHeading(new Pose2d(30, 0, 180), 0)
                .splineToConstantHeading(new Vector2d(40, 40), 0)
                .addDisplacementMarker(() -> {
                    testServo.setPosition(0.5);
                })
                .splineToConstantHeading(new Vector2d(0,0), 0)
                .splineTo(new Vector2d(10, -10), 0)
                .addSpatialMarker(new Vector2d(10,-10), () -> {
                    testServo.setPosition(0);
                })
                .build();

        drive.followTrajectory(ford);
    }
}
