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
   // public Servo flap = null;
    public float power = 0;
    public boolean toggle = true;
    public float power1 = 0;
    public boolean toggle1 = true;
    private DcMotor intake = null;
    private Servo kick = null;
    private Servo wheel = null;
    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;//  Used to control the right back drive wheel
    private Servo linear = null;
    double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    double  turn            = 0;// Desired turning power/speed (-1 to +1)
    double  ws = 0;
    double test = 0;



    @Override
    public void runOpMode() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftfront_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfront_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftback_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightback_drive");
        out = hardwareMap.get(DcMotor.class, "launchr");
        out1 = hardwareMap.get(DcMotor.class, "launchl");
       // flap = hardwareMap.get(Servo.class, "door");
        intake = hardwareMap.get(DcMotor.class, "intake");
        kick = hardwareMap.get(Servo.class, "kick");
        wheel = hardwareMap.get(Servo.class, "wheel");
        linear = hardwareMap.get(Servo.class, "linear");
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

//


        waitForStart();
        while (opModeIsActive()) {
            drive  = -gamepad1.left_stick_y ;  // Reduce drive rate to 50%.
            strafe = -gamepad1.left_stick_x ;  // Reduce strafe rate to 50%.
            turn   = gamepad1.right_stick_x;  // Reduce turn rate to 33%.

            telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            if(test > 0.5){
                intake.setPower(0);
            }
            if(test < 0.5){
                intake.setPower(gamepad2.right_stick_y);

            }

            out.setPower(-0.75*gamepad2.left_stick_y - ws);
            out1.setPower(0.75*gamepad2.left_stick_y + ws);
            if(gamepad1.a){
                ws=0.15;
            }else if (gamepad1.b){
                ws = 0;
            }
            telemetry.addData("Power", power);
            telemetry.update();
            if(gamepad2.left_bumper){
                if(test<0.5){
                    kick.setPosition(0.15);
                }
                if(test>0.5){
                    kick.setPosition(0.6);
                }
            }else{
                kick.setPosition(0.15);
                if(gamepad2.x){
                    wheel.setPosition(0);
                    test = 0;
                }
                    if(gamepad2.y){
                    wheel.setPosition(0.7272);
                    test = 0;
                }
                    if(gamepad2.b){
                    wheel.setPosition(0.384);
                    test = 0;
                }
                    if(gamepad2.x && gamepad2.right_bumper){
                    wheel.setPosition(0.565);
                    test = 1;
                    sleep(250);
                }
                    if(gamepad2.y && gamepad2.right_bumper){
                    wheel.setPosition(0.192);
                    test = 1;
                    sleep(250);
                }
                    if(gamepad2.b && gamepad2.right_bumper){
                    wheel.setPosition(0.909);
                    test = 1;
                    sleep(250);
                }
            }



     /*       if(gamepad2.a){
                flap.setPosition(0.4);
            } else{
                flap.setPosition(0.65);
            }   */

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

            telemetry.update();

            moveRobot(drive, strafe, turn);
            linear.setPosition(power);
            telemetry.addData("LINEAR", linear.getPosition());
            telemetry.addData("test", test);

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
        rightBackDrive.setPower(rightBackPower);
    }
}