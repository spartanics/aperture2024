package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class MarkersHWkrishna extends LinearOpMode {
    private Servo test;
    static final double full = 1;
    static final double empty = 0;
    static final double three4ths = 0.75;
    static final double thirty = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        test = hardwareMap.get(Servo.class, "test");
        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(48,0 ))
                .addDisplacementMarker(() -> {
                    test.setPosition(full);
                })

                .splineTo(new Vector2d(48, -24), Math.toRadians(0))

                .addDisplacementMarker(30, () -> {
                    test.setPosition(thirty);
                })
                .splineTo(new Vector2d(12, -12), Math.toRadians(-90))

                .addTemporalMarker(1, () -> {
                    test.setPosition(three4ths);
                })
                .splineTo(new Vector2d(24, -24), Math.toRadians(-90))

                .addSpatialMarker(new Vector2d(24, -24), () -> {

                    test.setPosition(empty);
                })
                .splineToLinearHeading(new Pose2d(36, -36, Math.toRadians(-90)), Math.toRadians(0))
                .build();

        drive.followTrajectory(traj);
    }
}
