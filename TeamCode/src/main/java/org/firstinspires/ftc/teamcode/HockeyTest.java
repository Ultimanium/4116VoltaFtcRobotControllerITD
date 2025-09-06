package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name="HockeyTest", group="Linear OpMode")

public class HockeyTest extends LinearOpMode {
    private Servo hockey = null;
    @Override
    public void runOpMode() {
        hockey = hardwareMap.get(Servo.class, "hockey");
        waitForStart();
        while (opModeIsActive()) {
            hockey.setPosition(gamepad1.right_stick_y);
        }
    }
}