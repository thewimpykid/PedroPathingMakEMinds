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

@Autonomous(name="BasketAuto")
public class BasketAuto extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(0, 112, Math.toRadians(270));
    private final Pose bucketPose = new Pose(4.5, 125.5, Math.toRadians(315));

    private final Pose sampleOnePose = new Pose(34.5, 118, Math.toRadians(0));
    private Path path, goToSample1;
    private Claw claw;
    private Slide slide;
    private Arm arm;

    public void buildPaths() {
        // Build the hangSpecimen1 PathChain
        path = new Path(new BezierLine(
                new Point(startPose),
//                new Point(44.000, 119.000, Point.CARTESIAN),
                new Point(bucketPose)));
        path.setLinearHeadingInterpolation(startPose.getHeading(), bucketPose.getHeading());
        path.setPathEndVelocityConstraint(0);
        path.setZeroPowerAccelerationMultiplier(1.5);
        // Build the goToSamples PathChain

    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                // Follow the hangSpecimen1 PathChain
                follower.followPath(path, true);  // holdEnd is true to allow corrections
//                arm.setPosition(-3750, 1.0); // this is the old arm position
                arm.setPosition(-2550, 1.0);
                slide.setPosition(-3000, 1.0);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy() && arm.sendPosition() < -2500) {
                    claw.setArmPosition(1.0);
                    claw.setClawPosition(0.7);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    claw.setArmPosition(0.475);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    arm.setPosition(700, 1.0);
                    slide.setPosition(-1000, 1.0);
                    setPathState(-1);
                }
//                follower.followPath(goToSample1);

                break;

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
        claw.setArmPosition(1.0);

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
