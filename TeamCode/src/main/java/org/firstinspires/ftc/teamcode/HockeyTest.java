package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Voltacular", group="Linear OpMode")

public class HockeyTest extends LinearOpMode {

    // Nathaniel's play area

    final double DESIRED_DISTANCE = 12.0;

    final double SPEED_GAIN  =  0.02  ;
    final double STRAFE_GAIN =  0.015 ;
    final double TURN_GAIN   =  0.01  ;

    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_STRAFE= 0.5;
    final double MAX_AUTO_TURN  = 0.3;

    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1;

    // below vars no touchy >:(

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    // Nathaniel's play area end

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
    double  ws = 0;
    double test = 0;



    @Override
    public void runOpMode() {

        // still no touchy.
        boolean targetFound = false;
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        // Let there be light.
        initAprilTag();

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

        //ashbaby
        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        waitForStart();
        while (opModeIsActive()) {
            targetFound = false;
            desiredTag  = null;
            //womp, womp.

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Is ts real?
                if (detection.metadata != null) {
                    // Do we want ts?
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // yuh uh
                        targetFound = true;
                        desiredTag = detection;
                        break;  // stop your gaze.
                    } else {
                        // nuh uh
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // ts does not exist :(
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Yell at driver
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.left_bumper && targetFound) {

                double headingError = 0;

                if(desiredTag.ftcPose.x == 6){
                    headingError = 90 + Math.atan(desiredTag.ftcPose.y / (desiredTag.ftcPose.x - 6));
                }

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = -gamepad1.left_stick_y;
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = gamepad1.right_stick_x;

                telemetry.addData("Auto","April tag distance", drive);
                telemetry.addData("Auto","ChangingAngle", headingError);
            } else {

                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                drive  = -gamepad1.left_stick_y;
                strafe = -gamepad1.left_stick_x;
                turn   = gamepad1.right_stick_x;
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }

            if(test > 0.5){
                intake.setPower(0);
            }

            if(test < 0.5){
                intake.setPower(gamepad2.right_stick_y);
            }

            out.setPower(-0.68*gamepad2.left_stick_y - ws);
            out1.setPower(0.68*gamepad2.left_stick_y + ws);
            if(gamepad1.a){
                ws=-0.15;
            }else if (gamepad1.b){
                ws = 0;
            }
            telemetry.addData("Power", power);
            telemetry.update();
            if(gamepad2.left_bumper){
                if(test<0.5){
                    kick.setPosition(0.15);
                    sleep(250);
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



            moveRobot(drive, strafe, turn);
            linear.setPosition(power);
            telemetry.addData("LINEAR", linear.getPosition());
            telemetry.addData("test", test);
            telemetry.update();
            sleep(10);
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

    //let there be a part two; electric boogaloo
    private void initAprilTag() {
        // On the first day, Nathaniel made the april tag holder
        aprilTag = new AprilTagProcessor.Builder().build();

        // On the second day. Nathaniel copy-pasted safety notices with quality of image results
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Made to be changed. Default is 2.
        aprilTag.setDecimation(2);

        // On the third day, Nathaniel made the vision portal
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
        // On the fourth, fifth, sixth, and seventh day, he passed out.
    }

    private void setManualExposure(int exposureMS, int gain) {
        // One must imagine the null loop happy

        if (visionPortal == null) {
            return;
        }

        // Am I on?
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Stop yo shit
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}