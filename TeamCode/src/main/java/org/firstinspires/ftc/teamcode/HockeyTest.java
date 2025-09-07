package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="HockeyTest", group="Linear OpMode")

public class HockeyTest extends LinearOpMode {
    private DcMotor left = null;
    private DcMotor right = null;
    private Servo push = null;
    private DcMotor in = null
    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        push = hardwareMap.get(Servo.class, "push");
        in = hardwareMap.get(DcMotor.class, "in");

        waitForStart();
        while (opModeIsActive()) {
            right.setPower(0.3);
            left.setPower(-0.3);
            push.setPosition(gamepad2.left_stick_y);
            in.setPower(0.5);

        }
    }

}