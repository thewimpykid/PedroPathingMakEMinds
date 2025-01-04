package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystem.Claw;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

/**
 * This teleop showcases movement and control of arm, slide, and other motors using PedroPathing.
 * The robot is controlled with the gamepad, where the left and right joysticks control robot movement.
 *
 * @version 2.0, 11/28/2024
 */

@TeleOp(name = "Gamepad")
public class Gamepad extends LinearOpMode {
    private Follower follower;
    private boolean isArmDoing = false;
    double clawPosition = 0;
    double armPosition = 0.5;
    double wristPosition = 0.5;
    private final Pose startPose = new Pose(0, 0, 0);
    Claw claw;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initializing motors and pathing
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        claw = new Claw(hardwareMap);

        // Initializing the IMU for orientation tracking
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);

        // Initialize the follower (PedroPathing's movement logic)
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        waitForStart();

        if (isStopRequested()) return;

        // Set slide motor to use encoders and apply braking when idle
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        follower.startTeleopDrive();

        // Main control loop
        while (opModeIsActive()) {
            imu.resetYaw();  // Reset IMU yaw to track orientation

            // Update PedroPathing follower for robot movement
            // Forward/Backward movement, left/right, and turning control
            if (gamepad1.right_trigger != 0) {
                follower.setTeleOpMovementVectors(
                        -gamepad1.left_stick_y,  // Forward/Backward
                        -gamepad1.left_stick_x,  // Left/Right
                        -gamepad1.right_stick_x, // Turning
                        true                       // Robot-Centric Mode
                );
            } else {
                follower.setTeleOpMovementVectors(
                        -gamepad1.left_stick_y / 5,  // Forward/Backward
                        -gamepad1.left_stick_x / 5,  // Left/Right
                        -gamepad1.right_stick_x / 5, // Turning
                        true                       // Robot-Centric Mode
                );
            }
            follower.update();

            // Control arm motor using gamepad2
            if (gamepad2.right_stick_y != 0.0) {
                armMotor.setPower(gamepad2.right_stick_y);
            } else {
                armMotor.setPower(0);
            }

            if (gamepad2.left_stick_y != 0.0) {
                slideMotor.setPower(gamepad2.left_stick_y);
            } else {
                slideMotor.setPower(0);
            }

            // Claw position control
            if (gamepad2.y) {
                clawPosition = 0.3;
            }
            if (gamepad2.x) {
                clawPosition = 0.7;
            }

            if (gamepad2.left_bumper) {
                wristPosition += 0.01;
            }
            if (gamepad2.right_bumper) {
                wristPosition -= 0.01;
            }

            if (gamepad2.left_trigger > 0) {
                armPosition += 0.01;
            }
            if (gamepad2.right_trigger > 0) {
                armPosition -= 0.01;
            }

            // Set positions for the claw, wrist, and arm
            claw.setClawPosition(clawPosition);
            claw.setWristPosition(wristPosition);
            claw.setArmPosition(armPosition);

            // Display telemetry data for all positions
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Arm Motor Position", armMotor.getCurrentPosition());
            telemetry.addData("Slide Motor Position", slideMotor.getCurrentPosition());
            telemetry.addData("Claw Position", clawPosition);
            telemetry.addData("Wrist Position", wristPosition);
            telemetry.addData("Arm Position", armPosition);
            telemetry.update();
        }
    }
}