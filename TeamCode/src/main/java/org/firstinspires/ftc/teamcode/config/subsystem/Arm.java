package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm{
    private DcMotorEx arm;
    private String armName;

    public Arm(String armName, HardwareMap hardwareMap) {
        this.arm = hardwareMap.get(DcMotorEx.class, armName);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public void setPowerArm(double power) {
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(power);
    }

    public void setModeEncoder(){
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void setModeNoEncoder(){
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public DcMotor getMotor() {
        return arm;
    }


}