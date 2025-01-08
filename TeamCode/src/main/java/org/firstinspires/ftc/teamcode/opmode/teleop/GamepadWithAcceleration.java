package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.teamcode.config.subsystem.Arm;
import org.firstinspires.ftc.teamcode.config.subsystem.Claw;
import org.firstinspires.ftc.teamcode.config.subsystem.Slide;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

/**
 * This teleop showcases movement and control of arm, slide, and other motors using PedroPathing.
 * The robot is controlled with the gamepad, where the left and right joysticks control robot movement.
 *
 * @version 2.0, 11/28/2024
 */

@TeleOp(name = "GamepadWithAcceleration")
public class GamepadWithAcceleration extends LinearOpMode {

    private Follower follower;
    private boolean isArmDoing = false;

    double clawPosition = 0;
    double armPosition = 0.5;
    double wristPosition = 0.5;

    private final Pose observationPose = new Pose(5, 30, Math.toRadians(180));
    private final Pose startPose = new Pose(10, 10, Math.toRadians(180));

    Claw claw;
    Arm arm;
    Slide slide;

    // Variables to store current motor power
    double currentForwardPower = 0;
    double currentStrafePower = 0;
    double currentTurnPower = 0;

    // Acceleration rate
    double accelerationStep = 0.05;  // Adjust this value for faster or slower acceleration

    // Slow mode factor
    double slowModeFactor = 0.5;  // This controls how slow the robot moves when the right trigger is pressed

    @Override
    public void runOpMode() throws InterruptedException {


        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        waitForStart();

        if (isStopRequested()) return;
        follower.startTeleopDrive();

        arm = new Arm("armMotor", hardwareMap);
        slide = new Slide("slideMotor", hardwareMap);
        claw = new Claw(hardwareMap);


        while (opModeIsActive()) {
            double targetForwardPower = -gamepad1.left_stick_y;
            double targetStrafePower = -gamepad1.left_stick_x;
            double targetTurnPower = -gamepad1.right_stick_x;

            // Check if right trigger is pressed to activate slow mode
            if (gamepad1.right_trigger > 0) {
                targetForwardPower *= slowModeFactor;
                targetStrafePower *= slowModeFactor;
                targetTurnPower *= slowModeFactor;
            }

            // Apply smooth power change
            currentForwardPower = smoothPowerChange(currentForwardPower, targetForwardPower);
            currentStrafePower = smoothPowerChange(currentStrafePower, targetStrafePower);
            currentTurnPower = smoothPowerChange(currentTurnPower, targetTurnPower);

            follower.setTeleOpMovementVectors(
                    currentForwardPower * 0.8,
                    currentStrafePower * 0.8,
                    currentTurnPower,
                    true
            );
            follower.update();

            // Control arm, slide, claw, wrist
            if (gamepad2.right_stick_y != 0.0) {
                arm.setPower(gamepad2.right_stick_y);
            } else {
                arm.setPower(0);
            }

            if (gamepad2.left_stick_y != 0.0) {
                slide.setPower(gamepad2.left_stick_y);
            } else {
                slide.setPower(0);
            }

            if (gamepad2.y) {
                clawPosition = 0.7;
            }
            if (gamepad2.x) {
                clawPosition = 1.0;
            }

            if (gamepad2.left_bumper) {
                wristPosition += 0.03;
            }
            if (gamepad2.right_bumper) {
                wristPosition -= 0.03;
            }

            if (gamepad2.left_trigger > 0) {
                armPosition += 0.002;
            }
            if (gamepad2.right_trigger > 0) {
                armPosition -= 0.002;
            }

            if (gamepad1.left_bumper) {
                setLoadPosClaw();
            }

            if (gamepad1.right_bumper) {
                setScorePosClaw();
            }

            claw.setClawPosition(clawPosition);
            claw.setWristPosition(wristPosition);
            claw.setArmPosition(armPosition);

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Arm Motor Position", arm.sendPosition());
            telemetry.addData("Slide Motor Position", slide.sendPosition());
            telemetry.addData("Claw Position", clawPosition);
            telemetry.addData("Wrist Position", wristPosition);
            telemetry.addData("Arm Position", armPosition);
            telemetry.update();
        }
    }

    private double smoothPowerChange(double current, double target) {
        if (current < target) {
            current += accelerationStep;
            if (current > target) {
                current = target;
            }
        } else if (current > target) {
            current -= accelerationStep;
            if (current < target) {
                current = target;
            }
        }
        return current;
    }

    public void setLoadPosClaw() {
        clawPosition = (0.3);
        wristPosition = (0.8);
        armPosition = (0.488);
    }

    public void setScorePosClaw() {
        clawPosition=(1.0);
        wristPosition=(0.8);
        armPosition=(0.55);
    }
}
