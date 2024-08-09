package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class TestYohan extends LinearOpMode {
    private Pose2d myPose = null;
    private SampleMecanumDrive drive = null;
    private Servo servo = null;

    public TestYohan() {
        drive = new SampleMecanumDrive(hardwareMap);
        servo = hardwareMap.get(Servo.class, "test");
    }

    public void updateTelemetry() {
        myPose = drive.getPoseEstimate();

        telemetry.addData("x", myPose.getX());
        telemetry.addData("y", myPose.getY());
        telemetry.addData("heading", Math.toDegrees(myPose.getHeading()));
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(24)
                // run the servo
                .addDisplacementMarker(() -> {
                    servo.setPosition(0);
                })
                .splineTo(new Vector2d(10, 10), Math.toRadians(90))
                .forward(24)
                .addDisplacementMarker(() -> {
                    servo.setPosition(1);
                })
                .build();

        drive.followTrajectorySequence(traj);
    }
}
