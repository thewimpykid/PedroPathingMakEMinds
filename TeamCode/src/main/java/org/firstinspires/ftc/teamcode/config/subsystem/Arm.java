package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private DcMotorEx arm;
    private String armName;

    public Arm(String armName, HardwareMap hardwareMap) {
        this.arm = hardwareMap.get(DcMotorEx.class, armName);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public int sendPosition() {
        return arm.getCurrentPosition();
    }
    public void setPosition(int pos, double speed) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(speed);
    }

    public void resetArm() {
        arm.setTargetPosition(0);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(1.0);
    }




}