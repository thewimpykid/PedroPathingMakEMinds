package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "GOTOSTART")
public class GoToStart extends LinearOpMode {

//    DcMotor arm;
    @Override
    public void runOpMode() throws InterruptedException {

        // Set arm to run to position mode without resetting encoder

        waitForStart();

        DcMotor arm = hardwareMap.get(DcMotor.class, "armMotor");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set the target position for the arm (position 0)

        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Set motor power to move the arm to position 0
        arm.setPower(0.5); // Adjust power as necessary

        waitForStart();

        // Wait until the arm reaches the target position
        while (opModeIsActive() && arm.isBusy()) {
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("Target Position", arm.getTargetPosition());
            telemetry.update();
        }

        // Once the arm reaches the target, stop it
        arm.setPower(0);
    }
}
