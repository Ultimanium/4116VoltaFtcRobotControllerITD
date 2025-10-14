package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="HockeyTest", group="Linear OpMode")

public class HockeyTest extends LinearOpMode {
    private DcMotor out = null;
    private DcMotor out1 = null;
    public Servo flap = null;
    public Servo pivot = null;
    public float power = 0;
    public boolean toggle = true;
    public float power1 = 0;
    public boolean toggle1 = true;
    private DcMotor intake = null;

    @Override
    public void runOpMode() {
        out = hardwareMap.get(DcMotor.class, "launchr");
        out1 = hardwareMap.get(DcMotor.class, "launchl");
        flap = hardwareMap.get(Servo.class, "door");
        pivot = hardwareMap.get(Servo.class, "pivot");
        intake = hardwareMap.get(DcMotor.class, "intake");


        waitForStart();
        while (opModeIsActive()) {
            if(gamepad2.right_bumper){
                intake.setPower(0.4);
            }else{intake.setPower(0);}
            out.setPower(power);
            out1.setPower(-power);

            telemetry.addData("Power", power);
            telemetry.addData("Pivot", pivot);
            telemetry.update();

            if(gamepad2.a){
                flap.setPosition(0.4);
            } else{
                flap.setPosition(0.65);
            }

            if(gamepad2.dpad_down && toggle){
                toggle = false;
                power -= 0.05f;
                if(power < -1){
                    power = -1;
                }
            } else if(gamepad2.dpad_up && toggle){
                toggle = false;
                power += 0.05f;
                if(power > 1){
                    power = 1;
                }
            } else if ((!toggle) && (!gamepad2.dpad_up) && (!gamepad2.dpad_down)){
                toggle = true;
            }

            if(gamepad2.dpad_left && toggle1){
                toggle1 = false;
                power1 -= 0.05f;
                if(power1 < 0){
                    power1 = 0;
                }
            } else if(gamepad2.dpad_right && toggle1){
                toggle1 = false;
                power1 += 0.05f;
                if(power1 > 1){
                    power1 = 1;
                }
            } else if ((!toggle1) && (!gamepad2.dpad_left) && (!gamepad2.dpad_right)){
                toggle1 = true;
            }

            pivot.setPosition(power1);
        }
    }

}