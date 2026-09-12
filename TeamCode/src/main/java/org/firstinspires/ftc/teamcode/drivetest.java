package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="30hrtest", group="Linear OpMode")

public class drivetest extends LinearOpMode {
    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;
    private DcMotor in = null;
    private Servo kick = null;
    private Servo kick2 = null;
    private DcMotor out = null;
    private DcMotor out2 = null;

    @Override
    public void runOpMode() {
        double drive = 0;
        double strafe = 0;
        double turn = 0;
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        in = hardwareMap.get(DcMotor.class, "in");
        kick = hardwareMap.get(Servo.class,"kick");
        kick2 = hardwareMap.get(Servo.class,"kick2");
        out = hardwareMap.get(DcMotor.class, "out");
        out2 = hardwareMap.get(DcMotor.class, "out2");

        waitForStart();
        while (opModeIsActive()) {

            in.setPower(gamepad2.left_stick_y);
            if(gamepad2.a){
                kick.setPosition(90);
            }
            if(gamepad2.b){
                kick.setPosition(0);
            }
            if(gamepad2.dpad_up){
                out.setPower(1);
                out2.setPower(-1);
            }
            if(gamepad2.dpad_down){
                out.setPower(0);
                out2.setPower(0);
            }
            if(gamepad2.x){
            kick2.setPosition(90);
            }
            if(gamepad2.y){
                kick2.setPosition(0);
            }

            drive  = -gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;
            turn   = -gamepad1.right_stick_x/2;
            telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            telemetry.addData("intake power:", in.getPower());
            moveRobot(drive, strafe, turn);
        }

    }
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);}

}
