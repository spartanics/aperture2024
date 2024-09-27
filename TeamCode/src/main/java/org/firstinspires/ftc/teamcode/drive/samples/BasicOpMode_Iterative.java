/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.samples;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@TeleOp(name="Basic: Iterative OpMode", group="Iterative OpMode")
//@Disabled
public class BasicOpMode_Iterative extends OpMode
{

    public enum ServoRotateState {
        NORMAL,
        GO_LEFT,
        GO_RIGHT
    };

    public enum ClawState {
        NORMAL,
        WRIST_DOWN,
        CLAW_RELEASE,
        CLAW_CLOSE,
        WRIST_UP
    }

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    private Servo testServo = null;
    private Servo clawServo = null;
    private Servo wristServo = null;

    final double MAX_POWER = 0.3;
    final double SERVO_WAIT = 1.1;

    private IMU imu = null;

    private ElapsedTime stopWatchDrone = new ElapsedTime();
    private ElapsedTime stopWatchClaw = new ElapsedTime();
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);


    ServoRotateState servoState = ServoRotateState.NORMAL;
    ClawState clawState = ClawState.NORMAL;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");

        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        testServo = hardwareMap.get(Servo.class, "test");
        clawServo = hardwareMap.get(Servo.class, "claw");
        wristServo = hardwareMap.get(Servo.class, "wrist");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Setting the back drive to just cruise
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // heading controller limit
        headingController.setInputBounds(-Math.PI, Math.PI);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        wristServo.setPosition(0);
        clawServo.setPosition(0);
        testServo.setPosition(1);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        stopWatchClaw.reset();
        stopWatchDrone.reset();
        runtime.reset();

    }

    public void updateClaw(Gamepad gamepad) {
        switch (clawState) {
            case NORMAL:
                if (stopWatchClaw.time() > SERVO_WAIT) {
                    if (gamepad.y) {
                        wristServo.setPosition(1);
                        stopWatchClaw.reset();
                        clawState = ClawState.WRIST_DOWN;
                    }
                }
                break;

            case WRIST_DOWN:
                // reset to normal
                if (stopWatchClaw.time() > SERVO_WAIT) {
                    clawServo.setPosition(1);
                    stopWatchClaw.reset();
                    clawState = ClawState.CLAW_CLOSE;
                }
                break;

            case CLAW_CLOSE:
                if (stopWatchClaw.time() > SERVO_WAIT) {
                    if (gamepad.y) {
                        wristServo.setPosition(0);
                        stopWatchClaw.reset();
                        clawState = ClawState.WRIST_UP;
                    }
                }
                break;

            case WRIST_UP:
                // reset to normal
                if (stopWatchClaw.time() > SERVO_WAIT) {
                    clawServo.setPosition(0);
                    stopWatchClaw.reset();
                    clawState = ClawState.NORMAL;
                }
                break;
        }

    }

    public void updateDroneServo(Gamepad gamepad, double waitTime) {

        // return to normal if x is pressed again
        switch (servoState) {
            case NORMAL:
                if (stopWatchDrone.time() > waitTime) {
                    if (gamepad.x) {
                        testServo.setPosition(0);
                        // set timer and change state to WAIT
                        stopWatchDrone.reset();
                        servoState = ServoRotateState.GO_LEFT;
                    }
                }
                break;

            case GO_LEFT:
                // reset to normal
                if (stopWatchDrone.time() > waitTime) {
                    testServo.setPosition(1);
                    stopWatchDrone.reset();
                    servoState = ServoRotateState.GO_RIGHT;
                }
                break;

            case GO_RIGHT:
                // reset to normal
                if (stopWatchDrone.time() > waitTime) {
                    testServo.setPosition(0);
                    stopWatchDrone.reset();
                    servoState = ServoRotateState.GO_LEFT;
                }
                break;
        }

        if (gamepad.a && servoState != ServoRotateState.NORMAL) {
            testServo.setPosition(1);
            stopWatchDrone.reset();
            servoState = ServoRotateState.NORMAL;
        }

    }

    public void updateDrive(Gamepad gamepad1) {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
//        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
//        double lateral =  gamepad1.left_stick_x;
//        double yaw     =  gamepad1.right_stick_x;

        Vector2d input = new Vector2d(
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y
        ).rotated(-getHeading());

        double axial = input.getY();
        double lateral = input.getX();

        // calculate the rotation and correction
        double rotation = (
                gamepad1.right_stick_x
        );

        // set headingInput to rotation if gamepad shows activities
        // otherwise rotate to correct heading
        double yaw = rotation;
        if (Math.abs(rotation) <= 0.05) {
            yaw = (headingController.update(getHeading()) * 0.006);
//                    * DriveConstants.kV;
//                    * DriveConstants.TRACK_WIDTH;
        }

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > MAX_POWER) {
            leftFrontPower  /= max * MAX_POWER;
            rightFrontPower /= max * MAX_POWER;
            leftBackPower   /= max * MAX_POWER;
            rightBackPower  /= max * MAX_POWER;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);


        // update the heeading target
        if (Math.abs(rotation) > 0.05) {
            headingController.setTargetPosition(getHeading());
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        updateDrive(gamepad1);
        updateDroneServo(gamepad1, 1.2);
        updateClaw(gamepad1);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.RADIANS);
    }

}
