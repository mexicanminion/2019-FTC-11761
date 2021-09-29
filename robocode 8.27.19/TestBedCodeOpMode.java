package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "TestBedCodeOpMode", group = "Robotivity")
public class TestBedCodeOpMode extends OpMode {
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front left motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front right motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "Back left motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "Back right motor");
    }

    @Override
    public void loop() {
    frontRightMotor.setPower(gamepad1.right_stick_y);
    frontLeftMotor.setPower(gamepad1.left_stick_y);
    backRightMotor.setPower(gamepad1.right_stick_y);
    backLeftMotor.setPower(gamepad1.left_stick_y);
    }

}

