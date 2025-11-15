package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="30hrtest", group="Linear OpMode")

public class HangTest extends LinearOpMode {
    private Servo door = null;
    private CRServo intake = null;
    private CRServo worm = null;
    @Override
    public void runOpMode() {
        door = hardwareMap.get(Servo.class, "door");
        intake = hardwareMap.get(CRServo.class, "intake");
        worm = hardwareMap.get(CRServo.class, "worm");
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad2.left_bumper) {
                door.setPosition(40);
            }else{door.setPosition(0);}
            intake.setPower(1);
            if(gamepad2.right_bumper){
                worm.setPower(1);
            }else{worm.setPower(0);}

        }
    }
}