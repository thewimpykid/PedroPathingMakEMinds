package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.teamcode.config.subsystem.Arm;
import org.firstinspires.ftc.teamcode.config.subsystem.Claw;
import org.firstinspires.ftc.teamcode.config.subsystem.Slide;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@TeleOp(name = "GamepadSpecEnhanced", group = "TeleOp")
public class GamepadSpecEnhanced extends LinearOpMode {

    private Follower follower;
    private Claw claw;
    private Arm arm;
    private Slide slide;

    // Teleop state: 0 = manual; 9, 10, 11 = autonomous segments
    private int teleopPathState = 0;
    private Timer pathTimer;
    private int counter = 0;
    private double yPlace = 62.5;

    // Autonomous paths for sequence
    private Path pickMore, placeSpecimen2;

    // Starting poses for teleop (if used)
    private final Pose parkPose = new Pose(0, 57, Math.toRadians(180));
    private final Pose bucketPose = new Pose(6.5, 131.5, Math.toRadians(315));

    // Drivetrain slow mode factor
    double slowModeFactor = 0.25;

    // Servo position variables
    double clawPosition = 1.0;
    double armPosition = 0.5;
    double wristPosition = 0.5;
    double draggerPosition = 0.76;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize IMU for field-centric (if needed)

        // Initialize follower and set starting pose
        follower = new Follower(hardwareMap);
        follower.setStartingPose(parkPose);

        // Initialize subsystems
        arm = new Arm("armMotor", hardwareMap);
        slide = new Slide("slideMotor", hardwareMap);
        claw = new Claw(hardwareMap);
        // Set initial servo positions
        claw.setClawPosition(1.0);
        claw.setWristPosition(0.856);
        claw.setArmPosition(0.572);

        // Initialize timer and counters
        pathTimer = new Timer();
        counter = 0;
        yPlace = 58.5;
        // Build default autonomous paths
        buildPaths();

        waitForStart();
        if (isStopRequested()) return;
        follower.startTeleopDrive();

        while (opModeIsActive()) {
            // --- Manual Drivetrain Control ---
            double targetForward = -gamepad1.left_stick_y;
            double targetStrafe  = -gamepad1.left_stick_x;
            double targetTurn    = -gamepad1.right_stick_x;
            if (gamepad1.right_trigger > 0) {
                targetForward *= slowModeFactor;
                targetStrafe  *= slowModeFactor;
                targetTurn    *= slowModeFactor;
            }
            if (teleopPathState == 0) {
                follower.setTeleOpMovementVectors(targetForward * 0.8,
                        targetStrafe  * 0.8,
                        targetTurn,
                        false);
            }
            follower.update();

            // --- Autonomous Sequence Triggering ---
            if (teleopPathState == 0 && gamepad1.a) {
                setPathState(9);
            }
            if (teleopPathState != 0 && gamepad1.y) {
                follower.breakFollowing();
                follower.startTeleopDrive();
                teleopPathState = 0;
            }

            if (teleopPathState == 0 && gamepad1.y) {
                follower.startTeleopDrive();
            }

            // --- Autonomous Sequence State Machine ---
            switch (teleopPathState) {

                case 9:
                    follower.setPose(new Pose(4.75, 25,  Math.toRadians(180)));
                    // Case 10: Wait until follower is done; then reset slide and set claw, then move to case 11.
                    if (!follower.isBusy()) {
                        slide.resetSlide();
                        claw.setClawPosition(1.0);
                        setPathState(10);
                    } else if (follower.getVelocity().getMagnitude() < 1.0 &&
                            follower.getCurrentPath().getClosestPointTValue() > 0.8 &&
                            follower.isBusy()) {
                        follower.breakFollowing();
                        slide.resetSlide();
                        claw.setClawPosition(1.0);
                        setPathState(10);
                    }
                    break;
                case 10:
                    // Case 11: After 0.5 sec, adjust yPlace, set arm, and build a new placeSpecimen2 path.
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        if (!follower.isBusy()) {
                            yPlace = yPlace + 2;
                        }
                        arm.setPosition(-2600, 0.75);
                        placeSpecimen2 = new Path(new BezierLine(
                                new Point(4.75, 25, Point.CARTESIAN),
                                new Point(28.5, yPlace, Point.CARTESIAN)
                        ));
                        placeSpecimen2.setConstantHeadingInterpolation(Math.toRadians(180));
                        placeSpecimen2.setZeroPowerAccelerationMultiplier(2);
                        if (!follower.isBusy()) {

                            follower.followPath(placeSpecimen2, true);
                        }
                        if (follower.getPose().getX() > 10) {
                            slide.setPosition(-200, 0.5);
                            if (slide.sendPosition() < -150) {
                                setPathState(11); // End autonomous sequence, return to manual.
                            }
                        }
                        setScorePosClaw();
                    }
                    break;
                case 11:

                    if (!follower.isBusy()) {
                        slide.setPosition(-1000, 1.0);
//                    claw.setArmPosition(0.7);
                        setPathState(12);
                    } else if (follower.getVelocity().getMagnitude() < 1.0 && follower.getCurrentPath().getClosestPointTValue() > 0.8 && follower.isBusy()) {
                        follower.breakFollowing();
                        slide.setPosition(-1000, 1.0);
                        setPathState(12);
                    }
                    break;
                case 12:
                    // Case 9: If 0.25 sec elapsed, counter < 4, and slide is below threshold, then:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25 && counter < 4 && slide.sendPosition() < -950) {
                        arm.setPosition(250, 1.0);
                        slide.setPosition(0, 1.0);
                        setPopulatePosClaw();
                        counter++;
                        // Build pickMore path dynamically (if needed, update parameters)
                        pickMore = new Path(new BezierLine(
                                new Point(34.0, 65.0, Point.CARTESIAN),
                                new Point(5.5, 26, Point.CARTESIAN)
                        ));
                        pickMore.setConstantHeadingInterpolation(Math.toRadians(180));
                        pickMore.setZeroPowerAccelerationMultiplier(2);
                        follower.followPath(pickMore);
                        setPathState(13);
                    }
                    break;
                case 13:
                    if (!follower.isBusy()) {setPathState(0);  }

                    break;
                default:
                    // Manual mode (teleopPathState == 0)
                    break;
            }

            // --- Mechanism Controls (Arm, Slide, Claw) ---
            if (teleopPathState == 0) {
                if (gamepad2.right_stick_y != 0.0) {
                    if (arm.sendPosition() > -2750 || gamepad2.right_stick_y > 0.0)
                        arm.setPowerArm(gamepad2.right_stick_y);
                    else
                        arm.setPowerArm(0);
                } else{
                    arm.setPowerArm(0);
                }

                if (gamepad2.left_stick_y != 0.0) {
                    if (slide.sendPosition() > -1800 || gamepad2.left_stick_y > 0.0 || arm.sendPosition() < -1500)
                        slide.setPowerSlide(gamepad2.left_stick_y);
                    else
                        slide.setPowerSlide(0);
                } else {
                    slide.setPowerSlide(0.0001);
                }

                if (slide.sendPosition() < -1750 && arm.sendPosition() > -1500) {
                    slide.setPosition(-1700, 1.0);
                } else {
                    slide.setModeEncoder();
                }
            }

            // Claw and servo controls
            if (gamepad2.y) {
                clawPosition = 0.25;
            }
            if (gamepad2.x) {
                clawPosition = 1.0;
            }
            if (gamepad2.b) {
                slide.resetSlide();
            }
            if (gamepad2.right_bumper) {
                wristPosition += 0.01;
            }
            if (gamepad2.left_bumper) {
                wristPosition -= 0.01;
            }
            if (gamepad2.left_trigger > 0) {
                armPosition += 0.0015;
            }
            if (gamepad2.right_trigger > 0) {
                armPosition -= 0.0015;
            }
            if (armPosition > 0.6845) {
                armPosition = 0.6825;
            }
            if (armPosition < 0.573) {
                armPosition = 0.572;
            }
            if (wristPosition > 1.0) {
                wristPosition = 1.0;
            }
            if (wristPosition < 0.797) {
                wristPosition = 0.799;
            }

            if (gamepad1.left_bumper) {
                setLoadPosClaw();
            }
            if (gamepad1.left_trigger != 0) {
                setPopulatePosClaw();
            }
            if (gamepad1.right_bumper) {
                setScorePosClaw();
            }
            if (gamepad2.a) {
                slide.setPosition(-100, 1.0);
            }

            // Update claw servo positions
            claw.setClawPosition(clawPosition);
            claw.setWristPosition(wristPosition);
            claw.setArmPosition(armPosition);
            claw.setDraggerPosition(draggerPosition);

            // Telemetry
            Pose p = follower.getPose();
            telemetry.addData("Teleop State", teleopPathState);
            telemetry.addData("X", p.getX());
            telemetry.addData("Y", p.getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(p.getHeading()));
            telemetry.addData("Arm Pos", arm.sendPosition());
            telemetry.addData("Slide Pos", slide.sendPosition());
            telemetry.update();
        }
    }

    // Builds default autonomous paths.
    private void buildPaths() {
        // Define initial pickMore path
        pickMore = new Path(new BezierLine(
                new Point(34.0, 65.0, Point.CARTESIAN),
                new Point(5.5, 26, Point.CARTESIAN)
        ));
        pickMore.setConstantHeadingInterpolation(Math.toRadians(180));
        pickMore.setZeroPowerAccelerationMultiplier(1.25);

        // Define initial placeSpecimen2 path using current yPlace value
        placeSpecimen2 = new Path(new BezierLine(
                new Point(5.5, 26, Point.CARTESIAN),
                new Point(34, yPlace, Point.CARTESIAN)
        ));
        placeSpecimen2.setConstantHeadingInterpolation(Math.toRadians(180));
        placeSpecimen2.setZeroPowerAccelerationMultiplier(2);
    }

    public void setPathState(int pState) {
        teleopPathState = pState;
        pathTimer.resetTimer();
    }

    public void setLoadPosClaw() {
        clawPosition = 0.25;
        wristPosition = 0.85;
        armPosition = 0.6825;
    }

    public void setScorePosClaw() {
        clawPosition = 1.0;
        wristPosition = 0.856;
        armPosition = 0.572;
    }

    public void setPopulatePosClaw() {
        clawPosition = 0.25;
        wristPosition = 0.853;
        armPosition = 0.635;
    }

    public void setClawLoad() {
        claw.setClawPosition(0.25);
        claw.setWristPosition(0.853);
        claw.setArmPosition(0.635);
    }

    public void setClawPut() {
        claw.setClawPosition(1.0);
        claw.setWristPosition(0.856);
        claw.setArmPosition(0.572);
    }
}
