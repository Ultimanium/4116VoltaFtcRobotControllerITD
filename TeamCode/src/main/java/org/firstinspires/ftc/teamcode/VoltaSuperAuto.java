package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Voltacular", group="Autonomous", preselectTeleOp = "VoltacularOp")

public class VoltaSuperAuto extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    private DcMotor out = null;
    private DcMotor out1 = null;
    private DcMotor intake = null;
    private Servo kick = null;
    private Servo wheel = null;
    private DcMotor leftFrontDrive   = null;
    private DcMotor rightFrontDrive  = null;
    private DcMotor leftBackDrive    = null;
    private DcMotor rightBackDrive   = null;
    private Servo linear = null;

    int balls = 3;
    double[] ballArray = {-1,-1,-1};
    long delay = 0;

    public enum TEAM {
        BLUE("Blue", 20),
        RED("Red", 24);

        private final String string;

        private final int tagCode;

        TEAM(String string, int tagCode) {
            this.string = string;
            this.tagCode = tagCode;
        }

        public String getString() {
            return string;
        }

        public int getTagCode() {
            return tagCode;
        }
    }

    public TEAM currentTeam;

    @Override
    public void runOpMode() {
        initVars();
        if(USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        ballArray = new double[]{0.565, 0.192, 0.909};
        wheel.setPosition(0.35);
        waitForStart();
        while (opModeIsActive()) {
            if(balls > 0){
                shootBall();
            }
        }
    }

    public void shootBall(){
        telemetry.addData("LAUNCHING BALL NUM",3 - balls);
        telemetry.update();
        linear.setPosition(0.6);
        kick.setPosition(0.15);
        out.setPower(0.555);
        out1.setPower(-0.555);
        wheel.setPosition(ballArray[3 - balls]);
        sleep(500);
        kick.setPosition(0.6);
        sleep(400);
        kick.setPosition(0.15);
        sleep(350);
        out.setPower(0);
        out1.setPower(0);
        balls--;
    }

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

    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        currentTeam = TEAM.BLUE;
        /*
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
                position = VoltaAuto.START_POSITION.Left;
                while (gamepad1.left_bumper && !isStopRequested()) {
                    telemetry.addData("Remove your hand from the left bumper NOW.", "");
                    telemetry.update();
                }
            }
            if (gamepad1.right_bumper) {
                position = VoltaAuto.START_POSITION.Right;
                while (gamepad1.dpad_down && !isStopRequested()) {
                    telemetry.addData("Remove your hand from the right bumper NOW.", "");
                    telemetry.update();
                }
            }
            telemetry.update();
        }
        while (gamepad1.a && !isStopRequested()) {
            telemetry.addData("Remove your hand from the a button.", "");
            telemetry.update();
        }
         */
        while (!isStopRequested()) {
            telemetry.addData("Initializing Autonomous Mode for Team ", "4116");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Set Team using X and B buttons on gamepad 1", "");
            telemetry.addData("Setting Team to  ", currentTeam.getString());
            telemetry.addData("\n press A on gamepad 1 to confirm", "");
            if (gamepad1.a) {
                break;
            }
            if (gamepad1.x) {
                currentTeam = TEAM.BLUE;
                while (gamepad1.x && !isStopRequested()) {
                    telemetry.addData("Remove your hand from the x button.", "");
                    telemetry.update();
                }
            }
            if (gamepad1.b) {
                currentTeam = TEAM.RED;
                while (gamepad1.b && !isStopRequested()) {
                    telemetry.addData("Remove your hand from the b button.", "");
                    telemetry.update();
                }
            }
            telemetry.update();
        }
    }

    public void selectStartingDelay() {
        while (gamepad1.a && !isStopRequested()) {
            telemetry.addData("Remove your hand from the a button.", "");
            telemetry.update();
        }
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        delay = 0;
        while (!isStopRequested()) {
            telemetry.addData("Initializing Autonomous Mode for Team ", "4116");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Set Delay using dpad on gamepad 1", "");
            telemetry.addData("Setting Delay to  ", delay);
            telemetry.addData("\n press A on gamepad 1 to confirm", "");
            if (gamepad1.a) {
                break;
            }
            if (gamepad1.dpad_up) {
                delay += 500;
                while (gamepad1.dpad_up && !isStopRequested()) {
                    telemetry.addData("Remove your hand from the dpad.", "");
                    telemetry.update();
                }
            }
            if (gamepad1.dpad_down) {
                delay -= 500;
                while (gamepad1.dpad_down && !isStopRequested()) {
                    telemetry.addData("Remove your hand from the dpad.", "");
                    telemetry.update();
                }
            }
            telemetry.update();
        }
        telemetry.update();
        telemetry.addData("Initializing Autonomous Mode for Team ", "4116");
        telemetry.addData("---------------------------------------", "");
        telemetry.addData("Team is set to  ", currentTeam.getString());
        telemetry.addData("Delay is set to  ", delay);
        telemetry.addData("\n all ready to go! Good luck, drive team!", "");
        telemetry.update();
    }

    public void initVars(){
        initAprilTag();

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        out = hardwareMap.get(DcMotor.class, "lr");
        out1 = hardwareMap.get(DcMotor.class, "ll");
        intake = hardwareMap.get(DcMotor.class, "i");
        kick = hardwareMap.get(Servo.class, "k");
        wheel = hardwareMap.get(Servo.class, "pw");
        linear = hardwareMap.get(Servo.class, "li");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        selectStartingPosition();
        selectStartingDelay();
    }
}
