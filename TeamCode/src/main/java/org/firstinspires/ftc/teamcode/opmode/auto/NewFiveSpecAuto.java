package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="NewFiveSpecAuto")
public class NewFiveSpecAuto extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose observationPose = new Pose(5.75, 30, Math.toRadians(180));
    private final Pose startPose = new Pose(0, 50, Math.toRadians(180));
    private final Pose chamberPose = new Pose(27, 60, Math.toRadians(180));
    private Claw claw;
    private Slide slide;
    private Arm arm;
    private int counter = 0;
    private int yPlace = 65;
    private PathChain hangSpecimen1, goToSamples;
    private Path goBack, pickSpecimen, placeSpecimen, placeSpecimen2, pickMore;

    public void buildPaths() {
        hangSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(chamberPose))) // First path
                .setLinearHeadingInterpolation(startPose.getHeading(), chamberPose.getHeading())// Heading interpolation
//                .setZeroPowerAccelerationMultiplier(1.25)
                .build();

        goBack = new Path(
                new BezierLine(
                        new Point(follower.getPose())
                        ,new Point(19, 59, Point.CARTESIAN)));
        goBack.setConstantHeadingInterpolation(chamberPose.getHeading());
//        goBack.setZeroPowerAccelerationMultiplier(1.25);

        pickSpecimen = new Path(new BezierLine(new Point(follower.getPose()), new Point(observationPose)));
        pickSpecimen.setPathEndVelocityConstraint(0);

        placeSpecimen = new Path(new BezierCurve(new Point(4.75, 25, Point.CARTESIAN),
                new Point(20, 10, Point.CARTESIAN),
                new Point(10, 45, Point.CARTESIAN),
                new Point(28.5, yPlace, Point.CARTESIAN)));
        placeSpecimen.setConstantHeadingInterpolation(Math.toRadians(180));
//        placeSpecimen.setZeroPowerAccelerationMultiplier(1.25);

        placeSpecimen2 = new Path(new BezierLine(new Point(4.75, 25, Point.CARTESIAN), new Point(28.5, yPlace, Point.CARTESIAN)));
        placeSpecimen2.setConstantHeadingInterpolation(Math.toRadians(180));
//        placeSpecimen2.setZeroPowerAccelerationMultiplier(1.25);

        pickMore = new Path( new BezierCurve(
                new Point(34.000, 65.000, Point.CARTESIAN),
                new Point(13.000, 63.250, Point.CARTESIAN),
                new Point(34.000, 32.500, Point.CARTESIAN),
                new Point(4, 25, Point.CARTESIAN)
        ));
        pickMore.setConstantHeadingInterpolation(Math.toRadians(180));
//        pickMore.setZeroPowerAccelerationMultiplier(1.25);

        goToSamples = follower.pathBuilder()
                .addPath(new BezierCurve( // First path - Bezier curve
                        new Point(19.000, 59.000, Point.CARTESIAN),
                        new Point(18, 25, Point.CARTESIAN),
                        new Point(40, 36, Point.CARTESIAN),
                        new Point(34, 25.500, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .setPathEndVelocityConstraint(5)
//                .setZeroPowerAccelerationMultiplier(1.25)// Heading interpolation

                .addPath(new BezierLine(new Point(34, 25.5, Point.CARTESIAN), new Point(5, 22, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))

                .addPath(new BezierCurve( // First path - Bezier curve
                        new Point(5, 22, Point.CARTESIAN),
                        new Point(40, 32, Point.CARTESIAN),
                        new Point(47.5, 10, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndVelocityConstraint(5)

                .addPath(new BezierLine( // First path - Bezier curve
                        new Point(47.5, 10, Point.CARTESIAN),
                        new Point(10, 10, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndVelocityConstraint(5)

                .addPath(new BezierCurve( // First path - Bezier curve
                        new Point(10, 10, Point.CARTESIAN),
                        new Point(53, 15, Point.CARTESIAN),
                        new Point(51, 8, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndVelocityConstraint(5)

                .addPath(new BezierCurve( // First path - Bezier curve
                        new Point(51, 8, Point.CARTESIAN),
                        new Point(51, -2, Point.CARTESIAN),
                        new Point(12, 3, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndVelocityConstraint(5)
                // third specimen
                .build();




        // First path.
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                // Logic for this case without paths
                follower.followPath(hangSpecimen1, true);

                arm.setPosition(-2550, 1.0);
                slide.setPosition(-200, 1.0);
                setPathState(1); // Set pathState to 1 (you can modify this based on desired flow)
                break;
            case 1:
                if (!follower.isBusy()) {
                    slide.setPosition(-1250, 1.0);
//                    claw.setArmPosition(0.7);
                    setPathState(2);
                }
                break;
            case 2:
                if (slide.sendPosition() < -1050) {
                    claw.setClawPosition(0.5);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
//                    setClawLoad();
                    slide.setPosition(0, 1.0);
                    follower.followPath(goBack);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    arm.setPosition(275, 1.0);
                    setClawLoad();
                    follower.setMaxPower(1);
                    follower.followPath(goToSamples);
                    setPathState(5);
                };
                break;
            case 5:
                if (!follower.isBusy()) {
                    claw.setClawPosition(1.0);
                    setPathState(-6);

                }
                break;
            case 6:
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(placeSpecimen, true);
//                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    slide.setPosition(-700, 1.0);
                    arm.setPosition(-2600, 1.0);
                    setClawPut();
                    setPathState(7);
                }
                break;
            case 7:
            case 11:
                if (!follower.isBusy()) {
                    slide.setPosition(-1600, 1.0);
                    //                    claw.setArmPosition(0.7);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && counter < 3) {
                    arm.setPosition(275, 1.0);
                    slide.setPosition(0, 1.0);
                    setClawLoad();
                    counter++;
                    follower.followPath(pickMore);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    claw.setClawPosition(1.0);
                    setPathState(10);
                }
                break;
            case 10:
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    yPlace += 2;
                    follower.followPath(placeSpecimen2, true);
//                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    arm.setPosition(-2600, 1.0);
                    slide.setPosition(-700, 1.0);

                    setClawPut();

                    setPathState(11);
                }
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
        claw.setWristPosition(0.856);
        claw.setArmPosition(0.572);

        buildPaths(); // No paths to build

        waitForStart(); // ^^^^^^^^^^^^^^ on INIT

        opmodeTimer.resetTimer();
        setPathState(0);
        while (opModeIsActive()) {
            follower.update();
//            follower.getDashboardPoseTracker();
            autonomousPathUpdate();

            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }

    public void setClawLoad() {
        claw.setClawPosition(0.25);
        claw.setWristPosition(0.85);
        claw.setArmPosition(0.6825);
    }

    public void setClawPut() {
        claw.setClawPosition(1.0);
        claw.setWristPosition(0.856);
        claw.setArmPosition(0.572); // 0.475
    }


}
