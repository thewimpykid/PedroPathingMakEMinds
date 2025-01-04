package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Slide {
    private DcMotorEx slide;
    private String slideName;

    public Slide(String slideName, HardwareMap hardwareMap) {
        this.slide = hardwareMap.get(DcMotorEx.class, slideName);
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    public int sendPosition() {
        return slide.getCurrentPosition();
    }
    public void setPosition(int pos, double speed) {
        slide.setTargetPosition(pos);
        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slide.setPower(speed);
    }

    public void resetArm() {
        slide.setTargetPosition(0);
        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slide.setPower(1.0);
    }




}