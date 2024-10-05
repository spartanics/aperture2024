package org.firstinspires.ftc.teamcode.TestingIsVeryCool;

import android.util.Size;

import com.acmerobotics.roadrunner.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.lang.Math;
import java.util.List;

@TeleOp(name = "Centering stuff", group = "Jonathan Is Cool")
public class CenteringStuff extends OpMode {
    DcMotor leftFrontDrive,
    leftBackDrive,
    rightFrontDrive,
    rightBackDrive;

    private ElapsedTime runtime = new ElapsedTime();
    ColorBlobLocatorProcessor colorLocator;
    VisionPortal portal;
    double[] vector = new double[2];

    @Override
    public void init() {

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();
        portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

    }

    @Override
    public void loop() {
        double xaverage = 0;
        double yaverage = 0;
        if (colorLocator.getBlobs().size() != 0) {
            ColorBlobLocatorProcessor.Blob blob = colorLocator.getBlobs().get(0);
            xaverage += blob.getBoxFit().center.x;

            yaverage += blob.getBoxFit().center.y;
        }






        vector[0] = 0;
        vector[1] = 0;
        if (xaverage < 155) {
            //move robot left
            vector[0] -= 0.5;
        }
        if (xaverage > 165) {
            //move robot right
            vector[0] += 0.5;
        }
        if (yaverage < 115) {
            //move robot forward
            vector[1] += 0.5;
        }
        if (yaverage > 125) {
            //move robot back
            vector[1] -= 0.5;
        }
        if (xaverage == 0) {
            vector[0] = 0;

            vector[1] = 0;
        }
        double leftBackPower = (vector[1] - vector[0])/7.0;
        double rightBackPower = (vector[1] - vector[0])/7.0;
        leftBackDrive.setPower(Math.round(vector[1] - vector[0])/5.0);
        rightFrontDrive.setPower(Math.round(vector[1] - vector[0])/5.0);
        leftFrontDrive.setPower(Math.round(vector[1] + vector[0])/5.0);
        rightBackDrive.setPower(Math.round(vector[1] + vector[0])/5.0);
        telemetry.addData("vectorx", vector[0]);
        telemetry.addData("vectory", vector[1]);
        telemetry.addData("x average",  xaverage);
        telemetry.addData("y average",  yaverage);
        telemetry.addData("leftbackpower", leftBackPower);
        telemetry.addData("rightbackpower", rightBackPower);
        telemetry.update();

    }

    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void stop() {
    }
}

