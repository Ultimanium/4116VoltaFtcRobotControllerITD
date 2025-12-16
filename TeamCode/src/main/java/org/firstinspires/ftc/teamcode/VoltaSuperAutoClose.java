package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Voltacular NOCAM FRONT", group="Autonomous", preselectTeleOp = "VoltacularOp")

public class VoltaSuperAutoClose extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)

            .setDrawTagOutline(true)
            .setLensIntrinsics(465.092,465.092,336.254,249.854)
            .build();

    private AprilTagDetection desiredTag = null;
    //public AprilTagProcessor.Builder builder = new AprilTagProcessor.Builder();
    public final Pose2d TAGPOS = new Pose2d(58.66142, 55.90551, 180 - 54.046);

    private boolean targetFound = false;

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
        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        ballArray = new double[]{0.565, 0.192, 0.938};
        wheel.setPosition(0.35);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose;
        Pose2d endPose;
        Pose2d shootPosition;
        if(currentTeam == TEAM.BLUE){
            startPose = new Pose2d(0, 0, 0);
            endPose = new Pose2d(48, 25, Math.toRadians(180));
            shootPosition = new Pose2d(48, 0, Math.toRadians(180));
        } else {
            startPose = new Pose2d(0, 0, 0);
            endPose = new Pose2d(48, -25, Math.toRadians(-180));
            shootPosition = new Pose2d(8, 0, Math.toRadians(-180));
        }

        drive.setPoseEstimate(startPose);

        Trajectory toShoot = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(shootPosition)
                .build();
        Trajectory end = drive.trajectoryBuilder(toShoot.end())
                .lineToLinearHeading(endPose)
                .build();

        waitForStart();





        // Step through the list of detected tags and look for a matching tag
        /*
        while (desiredTag == null && !isStopRequested()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if (detection.id == currentTeam.getTagCode()) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;

                        double relX = desiredTag.ftcPose.x;
                        double relY = desiredTag.ftcPose.y;
                        double hypo = Math.sqrt(Math.pow(relX, 2) + Math.pow(relY, 2));
                        double relYaw = Math.toRadians(desiredTag.ftcPose.yaw);
                        double angle1;
                        if(currentTeam == TEAM.BLUE){
                            angle1 = relYaw - Math.toRadians(TAGPOS.getHeading());
                        } else {
                            angle1 = relYaw + Math.toRadians(TAGPOS.getHeading());
                        }
                        double x = TAGPOS.getX() + (Math.cos(angle1) * hypo);
                        double y = TAGPOS.getY() + (Math.sin(angle1) * hypo);
                        double yaw = Math.atan(relX/relY) - (Math.PI/2) + angle1;


                        telemetry.addData("angle1", angle1);
                        telemetry.addData("hypo", hypo);
                        telemetry.addData("relX", relX);
                        telemetry.addData("x", x);
                        telemetry.addData("y", y);
                        telemetry.addData("rot", yaw);
                        telemetry.addData("relYaw", relYaw);
                        telemetry.update();
                        startPose = new Pose2d(x, y, yaw);

                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }
        }

         */

        out.setPower(0.555);
        out1.setPower(-0.555);

        drive.followTrajectory(toShoot);

        targetFound = false;
        desiredTag = null;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Is ts real?
            if (detection.metadata != null) {
                // Do we want ts?
                if (detection.id == 21){
                    // GPP
                    ballArray = new double[]{0.565, 0.192, 0.938};
                } else if (detection.id == 22){
                    // PGP
                    ballArray = new double[]{0.192, 0.565, 0.938};
                } else if (detection.id == 23){
                    // PPG
                    ballArray = new double[]{0.192, 0.938, 0.565};
                } else {
                    // nuh uh
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // ts does not exist :(
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        sleep(1000);

        while (balls > 0){
            shootBall();
        }

        sleep(1000);

        out.setPower(0);
        out1.setPower(-0);

        drive.followTrajectory(end);

        while (opModeIsActive()) {

            //List<AprilTagDetection> currentDetections = aprilTag.getDetections();



            //telemetry.update();

            drive.update();


        }
    }

    public void shootBall(){
        telemetry.addData("LAUNCHING BALL NUM",3 - balls);
        telemetry.update();
        linear.setPosition(0.6);
        kick.setPosition(0.15);
        wheel.setPosition(ballArray[3 - balls]);
        sleep(500);
        kick.setPosition(0.6);
        sleep(400);
        kick.setPosition(0.15);
        sleep(350);
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
