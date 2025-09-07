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
    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        waitForStart();
        while (opModeIsActive()) {
            right.setPower(0.5);
            left.setPower(-0.5);
        }
    }

}