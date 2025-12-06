package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="Voltacular (DEFUNCT)", group="Autonomous", preselectTeleOp = "VoltacularOp")

public class VoltaAuto extends LinearOpMode {

    // Nathaniel's play area

    final double DESIRED_DISTANCE = 48.0;

    final double SPEED_GAIN  =  1  ;
    final double STRAFE_GAIN =  1 ;
    final double EXPONENTIAL_RANGE = 12;
    final double TURN_GAIN   =  1  ;
    final double EXPONENTIAL_TURN_RANGE = 25;

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
    private DcMotor intake = null;
    private Servo kick = null;
    private Servo wheel = null;
    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;//  Used to control the right back drive wheel
    private Servo linear = null;

    int desiredID = -1;

    int balls = 3;
    double[] ballArray = {-1,-1,-1};
    long delay = 0;

    public enum START_POSITION{
        Left,
        Right
    }

    public VoltaAuto.START_POSITION position;

    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        position = START_POSITION.Right;
        desiredID = 20;
        while (!isStopRequested()) {
            telemetry.addData("Initializing Autonomous Mode for Team ", "4116");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Set Position using bumpers on gamepad 1:", "");
            telemetry.addData("Setting Position to  ", position);
            telemetry.addData("\n press A on gamepad 1 to confirm", "");
            if (gamepad1.a) {
                break;
            }
            if (gamepad1.left_bumper) {
                position = START_POSITION.Left;
                while (gamepad1.left_bumper && !isStopRequested()) {
                    telemetry.addData("Remove your hand from the left bumper NOW.", "");
                    telemetry.update();
                }
            }
            if (gamepad1.right_bumper) {
                position = START_POSITION.Right;
                while (gamepad1.dpad_down && !isStopRequested()) {
                    telemetry.addData("Remove your hand from the right bumper NOW.", "");
                    telemetry.update();
                }
            }
            telemetry.update();
        }
        while (gamepad1.a && !isStopRequested()) {
            telemetry.addData("Remove your hand from the a button NOW.", "");
            telemetry.update();
        }
        while (!isStopRequested()) {
            telemetry.addData("Initializing Autonomous Mode for Team ", "4116");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Set Team using buttons on gamepad 1:", "");
            telemetry.addData("Setting Team to  ", desiredID);
            telemetry.addData("20 = Blue, 24 = Red", "");
            telemetry.addData("\n press A on gamepad 1 to confirm", "");
            if (gamepad1.a) {
                break;
            }
            if (gamepad1.x) {
                desiredID = 20;
                while (gamepad1.x && !isStopRequested()) {
                    telemetry.addData("Remove your hand from the x button NOW.", "");
                    telemetry.update();
                }
            }
            if (gamepad1.b) {
                desiredID = 24;
                while (gamepad1.b && !isStopRequested()) {
                    telemetry.addData("Remove your hand from the b button NOW.", "");
                    telemetry.update();
                }
            }
            telemetry.update();
        }
    }

    public void selectStartingDelay() {
        while (gamepad1.a && !isStopRequested()) {
            telemetry.addData("Remove your hand from the a button NOW.", "");
            telemetry.update();
        }
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        delay = 0;
        while (!isStopRequested()) {
            telemetry.addData("Initializing Autonomous Mode for Team ", "4116");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Set Delay using dpad on gamepad 1:", "");
            telemetry.addData("Setting Delay to  ", delay);
            telemetry.addData("\n press A on gamepad 1 to confirm", "");
            if (gamepad1.a) {
                break;
            }
            if (gamepad1.dpad_up) {
                delay += 500;
                while (gamepad1.dpad_up && !isStopRequested()) {
                    telemetry.addData("Remove your hand from the dpad NOW.", "");
                    telemetry.update();
                }
            }
            if (gamepad1.dpad_down) {
                delay -= 500;
                while (gamepad1.dpad_down && !isStopRequested()) {
                    telemetry.addData("Remove your hand from the dpad NOW.", "");
                    telemetry.update();
                }
            }
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() {

        // still no touchy.
        boolean targetFound = false;
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        // Let there be light.
        initAprilTag();

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        out = hardwareMap.get(DcMotor.class, "lr");
        out1 = hardwareMap.get(DcMotor.class, "ll");
       // flap = hardwareMap.get(Servo.class, "door");
        intake = hardwareMap.get(DcMotor.class, "i");
        kick = hardwareMap.get(Servo.class, "k");
        wheel = hardwareMap.get(Servo.class, "pw");
        linear = hardwareMap.get(Servo.class, "li");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //ashbaby
        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        selectStartingPosition();
        selectStartingDelay();

        wheel.setPosition(0.35);
        waitForStart();
        linear.setPosition(0.6);
        sleep(delay);
        moveRobot(0.75,0,0);
        sleep(1500);
        moveRobot(0,0,0);
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
                    if ((detection.id == desiredID) && ballArray[0] != -1) {
                        // yuh uh
                        targetFound = true;
                        desiredTag = detection;
                        break;  // stop your gaze.
                    } else if (detection.id == 21){
                        // GPP
                        ballArray = new double[]{0.565, 0.192, 0.909};
                    } else if (detection.id == 22){
                        // PGP
                        ballArray = new double[]{0.192, 0.565, 0.909};
                    } else if (detection.id == 23){
                        // PPG
                        ballArray = new double[]{0.192, 0.909, 0.565};
                    } else {
                        // nuh uh
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // ts does not exist :(
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            if (targetFound) {

                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = 0;
                double  yawError        = desiredTag.ftcPose.yaw;

                if(desiredTag.ftcPose.x != 6){
                    headingError = ((180 / Math.PI) * Math.atan(desiredTag.ftcPose.y / (desiredTag.ftcPose.x + 6)));
                }


                if(headingError < 0){
                    headingError = -(-90 - headingError);
                } else if(headingError > 0) {
                    headingError = -(90 - headingError);
                }


                if((Math.abs(headingError) < 5 && Math.abs(rangeError) < 3  && Math.abs(yawError) < 3) && balls > 0){
                    telemetry.addData("LAUNCHING BALL NUM",3 - balls);
                    telemetry.update();
                    linear.setPosition(0.6);
                    kick.setPosition(0.15);
                    out.setPower(0.54);
                    out1.setPower(-0.54);
                    moveRobot(0, 0, 0);
                    wheel.setPosition(ballArray[3 - balls]);
                    sleep(1000);
                    kick.setPosition(0.6);
                    sleep(1000);
                    kick.setPosition(0.15);
                    sleep(1000);
                    out.setPower(0);
                    out1.setPower(0);
                    balls--;
                } else {
                    // Use the speed and turn "gains" to calculate how we want the robot to move.
                    drive  = Range.clip(Math.pow(rangeError / EXPONENTIAL_RANGE, 3) * EXPONENTIAL_RANGE * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn   = Range.clip(Math.pow(headingError / EXPONENTIAL_TURN_RANGE, 3) * EXPONENTIAL_TURN_RANGE * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    strafe = Range.clip(Math.pow(-yawError / EXPONENTIAL_RANGE, 3) * EXPONENTIAL_RANGE * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                    telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                    telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                    telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                    telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                    telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
                    telemetry.addData("April tag distance", desiredTag.ftcPose.y);
                    telemetry.addData("changing angle",headingError);
                }
            } else {

                if(desiredID == 20){
                    drive  = 0;
                    strafe = 0;
                    turn   = 0.5;
                } else {
                    drive  = 0;
                    strafe = 0;
                    turn   = -0.5;
                }
                telemetry.addData("Searching...","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            moveRobot(drive, strafe, turn);
            telemetry.update();
            sleep(10);

            if(balls == 0){
                if(desiredID == 20){
                    moveRobot(0,1,0);
                    sleep(2000);
                    moveRobot(0,0,0);
                    break;
                } else {
                    moveRobot(0,-1,0);
                    sleep(2000);
                    moveRobot(0,0,0);
                    break;
                }
            }
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
        leftFrontDrive.setPower(leftFrontPower/2);
        rightFrontDrive.setPower(rightFrontPower/2);
        leftBackDrive.setPower(leftBackPower/2);
        rightBackDrive.setPower(rightBackPower/2);
    }

    public void moveRobot(double x, double y, double yaw, long milleseconds) {
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

        sleep(milleseconds);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
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