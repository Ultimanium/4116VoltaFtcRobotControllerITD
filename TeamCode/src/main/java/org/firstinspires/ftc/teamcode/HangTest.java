package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="HangTest", group="Linear OpMode")

public class HangTest extends LinearOpMode {
    private DcMotor hang = null;
    @Override
    public void runOpMode() {
        hang = hardwareMap.get(DcMotor.class, "hang");
        waitForStart();
        while (opModeIsActive()) {
            hang.setPower(gamepad1.right_stick_y);
        }
    }
}