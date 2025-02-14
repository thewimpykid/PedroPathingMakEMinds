package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.config.subsystem.Arm;

@Autonomous(name = "ArmReset")
public class GoToStart extends LinearOpMode {

    Arm arm = new Arm("armMotor", hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {

        // Set arm to run to position mode without resetting encoder

        waitForStart();


        arm.resetArm();


        // Set the target position for the arm (position 0)

        arm.setPosition(200, 0.25);
        // Adjust power as necessary


        // Wait until the arm reaches the target position
        while (opModeIsActive()) {
            telemetry.addData("Arm Position", arm.sendPosition());
            telemetry.update();
        }

        // Once the arm reaches the target, stop it
    }
}
