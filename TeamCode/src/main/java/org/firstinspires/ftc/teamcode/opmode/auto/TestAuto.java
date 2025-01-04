package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.subsystem.Arm;
import org.firstinspires.ftc.teamcode.config.subsystem.Claw;
import org.firstinspires.ftc.teamcode.config.subsystem.Slide;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name="TestAuto")
public class TestAuto extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(0, 50, Math.toRadians(270));
    private final Pose chamberPose = new Pose(31, 65, Math.toRadians(180));

    private PathChain hangSpecimen1, goToSamples,  takeSample3;
    private Claw claw;
    private Slide slide;
    private Arm arm;

    public void buildPaths() {
        // Build the hangSpecimen1 PathChain
        hangSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(chamberPose))) // First path
                .setLinearHeadingInterpolation(startPose.getHeading(), chamberPose.getHeading()) // Heading interpolation
                .build();


        // Build the goToSamples PathChain
        goToSamples = follower.pathBuilder()
                .addPath(new BezierLine(new Point(chamberPose), new Point(20, 65, Point.CARTESIAN))) // First path
                .setConstantHeadingInterpolation(Math.toRadians(180)) // Heading interpolation
                .setPathEndVelocityConstraint(0)
                .addPath(new BezierCurve( // First path - Bezier curve
                        new Point(20.000, 65.000, Point.CARTESIAN),
                        new Point(14, 22, Point.CARTESIAN),
                        new Point(49.000, 32.000, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180)) // Heading interpolation
                .setPathEndVelocityConstraint(0)
                .addPath(new BezierCurve( // First path - straight line
                        new Point(49.000, 32.000, Point.CARTESIAN),
                        new Point(59.000, 13, Point.CARTESIAN),
                        new Point(15.000, 14.000, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180)) // Heading interpolation
                .setPathEndVelocityConstraint(0)
                .addPath(new BezierCurve( // First path - straight line
                        new Point(15.000, 14.000, Point.CARTESIAN),
                        new Point(42.000, 38.000, Point.CARTESIAN),
                        new Point(49.000, 20.000, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180)) // Heading interpolation
                .setPathEndVelocityConstraint(0)
                .addPath(new BezierCurve( // First path - straight line
                        new Point(49.000, 20.000, Point.CARTESIAN),
                        new Point(65.000, 2.000, Point.CARTESIAN),
                        new Point(10.000, 10, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180)) // Heading interpolation
                .setPathEndVelocityConstraint(0)
                .build();

        takeSample3 = follower.pathBuilder()
                .addPath(new BezierCurve( // First path - straight line
                        new Point(10.000, 10.000, Point.CARTESIAN),
                        new Point(30.000, 17.000, Point.CARTESIAN),
                        new Point(40, 12, Point.CARTESIAN)
                ))
                .setPathEndVelocityConstraint(0)
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .addPath(new BezierLine(
                        new Point(40, 12, Point.CARTESIAN),
                        new Point(40, 4, Point.CARTESIAN)
                ))// Heading interpolation
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .setPathEndVelocityConstraint(0)
                .addPath(new BezierLine(
                    new Point(40, 4, Point.CARTESIAN),
                    new Point(0, 4, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .setPathEndVelocityConstraint(0)
                .build();

        // Build the takeSampleOne PathChain


    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                // Follow the hangSpecimen1 PathChain
                follower.followPath(hangSpecimen1, false);  // holdEnd is true to allow corrections
                arm.setPosition(-3750, 1.0);
                slide.setPosition(-700, 1.0);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    slide.setPosition(0, 1.0);
                    if (slide.sendPosition() > -200) {
                        claw.setClawPosition(0);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {// Follow the goToSamples PathChain
                    arm.setPosition(0, 1.0);
                    follower.followPath(goToSamples, false);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    arm.setPosition(-2000, 1.0);
                    follower.followPath(takeSample3, false);
                    setPathState(-1);
                }


        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void runOpMode() {
        claw = new Claw(hardwareMap);
        slide = new Slide("slideMotor", hardwareMap);
        arm = new Arm("armMotor", hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        claw.setClawPosition(1.0);
        claw.setWristPosition(0.8);
        claw.setArmPosition(0.9);

        buildPaths();

        waitForStart(); // ^^^^^^^^^^^^^^ on INIT

        opmodeTimer.resetTimer();
        setPathState(0);
        while (opModeIsActive()) {
            follower.update();
            follower.getDashboardPoseTracker();
            autonomousPathUpdate();

            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}
