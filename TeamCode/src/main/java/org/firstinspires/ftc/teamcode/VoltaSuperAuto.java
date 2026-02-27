package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Voltacular BACK", group="Autonomous", preselectTeleOp = "VoltacularOp")

public class VoltaSuperAuto extends LinearOpMode {

    private ElapsedTime lastIntake = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

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

    private DcMotorEx out = null;
    private DcMotorEx out1 = null;
    private DcMotor intake = null;
    private Servo kick = null;
    private Servo wheel = null;
    private DcMotor leftFrontDrive   = null;
    private DcMotor rightFrontDrive  = null;
    private DcMotor leftBackDrive    = null;
    private DcMotor rightBackDrive   = null;
    private Servo linear = null;
    private ColorSensor bcs = null;

    int balls = 3;
    double[] ballArray = {-1,-1,-1};
    double[] ballInputArray = {0,0.354,0.7272};
    long delay = 0;

    public float P = 35f;
    public float I = 0;
    public float D = 0.1f;
    public float F = 11.9f;
    double flywheelPower = 1400;

    public VoltacularOp.BALL[] Balls = {null,null,null};

    private VoltacularOp.BALL ProtoBall = VoltacularOp.BALL.GREEN;

    public int focusedBall = 0;

    public VoltacularOp.COLOR[] BallSequence = {null, null, null};

    public VoltaAuto.START_POSITION position;

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
        Pose2d shootPositionSecond;
        Pose2d inputStart;
        Pose2d[] inputs;
        if(currentTeam == TEAM.BLUE){
            startPose = new Pose2d(0, 0, 0);
            endPose = new Pose2d(1, 25, 0);
            shootPosition = new Pose2d(8, 0, Math.toRadians(22));
            shootPositionSecond = new Pose2d(8, -3, Math.toRadians(22));
            inputStart = new Pose2d(28, 5, -Math.toRadians(90));
            inputs = new Pose2d[] {new Pose2d(28, 16.5, -Math.toRadians(90)), new Pose2d(28, 21, -Math.toRadians(90)), new Pose2d(28, 27, -Math.toRadians(90))};
        } else {
            startPose = new Pose2d(0, 0, 0);
            endPose = new Pose2d(1, -25, 0);
            shootPosition = new Pose2d(8, 0, -Math.toRadians(24));
            shootPositionSecond = new Pose2d(8, 3, -Math.toRadians(24));
            inputStart = new Pose2d(28, -5, Math.toRadians(90));
            inputs = new Pose2d[] {new Pose2d(28, -16.5, Math.toRadians(90)), new Pose2d(28, -21, Math.toRadians(90)), new Pose2d(28, -27, Math.toRadians(90))};
        }

        drive.setPoseEstimate(startPose);

        Trajectory toShoot = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(shootPosition)
                .build();
        Trajectory toBalls = drive.trajectoryBuilder(toShoot.end())
                .lineToLinearHeading(inputStart)
                .build();
        Trajectory toBall1 = drive.trajectoryBuilder(toBalls.end())
                .lineToLinearHeading(inputs[0])
                .build();
        Trajectory toBall2 = drive.trajectoryBuilder(toBall1.end())
                .lineToLinearHeading(inputs[1])
                .build();
        Trajectory toBall3 = drive.trajectoryBuilder(toBall2.end())
                .lineToLinearHeading(inputs[2])
                .build();
        Trajectory[] allBalls = {
                toBall1,
                toBall2,
                toBall3
        };
        Trajectory toShootAgain = drive.trajectoryBuilder(toBall3.end())
                .lineToLinearHeading(shootPositionSecond)
                .build();
        Trajectory end = drive.trajectoryBuilder(toShootAgain.end())
                .lineToLinearHeading(endPose)
                .build();

        waitForStart();
        linear.setPosition(0.15);


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
                     BallSequence = new VoltacularOp.COLOR[]{VoltacularOp.COLOR.GREEN, VoltacularOp.COLOR.PURPLE, VoltacularOp.COLOR.PURPLE};
                     VoltacularOp.colorSequence = BallSequence;
                } else if (detection.id == 22){
                    // PGP
                    ballArray = new double[]{0.192, 0.565, 0.938};
                     BallSequence = new VoltacularOp.COLOR[]{VoltacularOp.COLOR.PURPLE, VoltacularOp.COLOR.GREEN, VoltacularOp.COLOR.PURPLE};
                     VoltacularOp.colorSequence = BallSequence;
                } else if (detection.id == 23){
                    // PPG
                    ballArray = new double[]{0.192, 0.938, 0.565};
                     BallSequence = new VoltacularOp.COLOR[]{VoltacularOp.COLOR.PURPLE, VoltacularOp.COLOR.PURPLE, VoltacularOp.COLOR.GREEN};
                     VoltacularOp.colorSequence = BallSequence;
                } else {
                    // nuh uh
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // ts does not exist :(
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

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

        drive.followTrajectoryAsync(toShoot);

        while(drive.isBusy()){
            drive.update();
            out1.setVelocity(flywheelPower);
            out.setVelocity(flywheelPower);
            PIDFCoefficients test2 = new PIDFCoefficients(P, I, D, F);
            out.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,test2);
            PIDFCoefficients test1 = new PIDFCoefficients(P, I, D, F);
            out1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,test1);
        }

        while (balls > 0){
            shootBall();
        }

        sleep(1000);
        wheel.setPosition(ballInputArray[0]);

        out.setPower(0);
        out1.setPower(-0);

        intake.setPower(1);
        drive.followTrajectory(toBalls);
        //drive.followTrajectoryAsync(allBalls[0]);
        moveRobot(-0.15,0,0);
        runtime.reset();
        while(balls < 3 && runtime.milliseconds() < 5000){
            telemetry.addData("touched", bcs.alpha() > 250);
            telemetry.addData("balls", balls);
            telemetry.update();
            drive.update();
            if(bcs.alpha() > 250 && lastIntake.milliseconds() > 600 && balls < 3){
                balls++;
                if(balls < 3){
                    moveRobot(-0.15 - (0.15 * balls),0,0);
                    wheel.setPosition(ballInputArray[balls]);
                    lastIntake.reset();
                    if(balls == 2){
                        moveRobot(0,0,0);
                        sleep(500);
                        moveRobot(-0.25 - (0.15 * balls),0,0);
                    }
                    //while(drive.isBusy()){
                        //drive.update();
                    //}
                    //drive.followTrajectoryAsync(allBalls[balls]);
                }
            }
        }
        moveRobot(0,0,0);
        balls = 3;

        sleep(1000);

        toShootAgain = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(shootPositionSecond)
                .build();
        end = drive.trajectoryBuilder(toShootAgain.end())
                .lineToLinearHeading(endPose)
                .build();

        drive.followTrajectoryAsync(toShootAgain);

        while(drive.isBusy()){
            drive.update();
            out1.setVelocity(flywheelPower);
            out.setVelocity(flywheelPower);
            PIDFCoefficients test2 = new PIDFCoefficients(P, I, D, F);
            out.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,test2);
            PIDFCoefficients test1 = new PIDFCoefficients(P, I, D, F);
            out1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,test1);
        }

        intake.setPower(0);

        while (balls > 0){
            shootBall();
        }

        sleep(1000);

        drive.followTrajectory(end);

        while (opModeIsActive()) {

            out.setPower(0);
            out1.setPower(0);

            //List<AprilTagDetection> currentDetections = aprilTag.getDetections();



            //telemetry.update();

            drive.update();


        }
    }

    public void shootBall(){
        telemetry.addData("LAUNCHING BALL NUM",3 - balls);
        telemetry.update();
        linear.setPosition(0.15);
        kick.setPosition(0.15);
        out1.setVelocity(flywheelPower);
        out.setVelocity(flywheelPower);
        PIDFCoefficients test2 = new PIDFCoefficients(P, I, D, F);
        out.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,test2);
        PIDFCoefficients test1 = new PIDFCoefficients(P, I, D, F);
        out1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,test1);
        wheel.setPosition(ballArray[3 - balls]);
        runtime.reset();
        while(runtime.milliseconds() < 500){
            out1.setVelocity(flywheelPower);
            out.setVelocity(flywheelPower);
            test2 = new PIDFCoefficients(P, I, D, F);
            out.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,test2);
            test1 = new PIDFCoefficients(P, I, D, F);
            out1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,test1);
        }
        kick.setPosition(0.6);
        runtime.reset();
        while(runtime.milliseconds() < 400){
            out1.setVelocity(flywheelPower);
            out.setVelocity(flywheelPower);
            test2 = new PIDFCoefficients(P, I, D, F);
            out.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,test2);
            test1 = new PIDFCoefficients(P, I, D, F);
            out1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,test1);
        }
        kick.setPosition(0.15);
        runtime.reset();
        while(runtime.milliseconds() < 350){
            out1.setVelocity(flywheelPower);
            out.setVelocity(flywheelPower);
            test2 = new PIDFCoefficients(P, I, D, F);
            out.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,test2);
            test1 = new PIDFCoefficients(P, I, D, F);
            out1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,test1);
        }
        out1.setVelocity(0);
        out.setVelocity(flywheelPower);
        test2 = new PIDFCoefficients(P, I, D, F);
        out.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,test2);
        test1 = new PIDFCoefficients(P, I, D, F);
        out1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,test1);
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
        out = hardwareMap.get(DcMotorEx.class, "lr");
        out1 = hardwareMap.get(DcMotorEx.class, "ll");
        out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        out1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        out.setDirection(DcMotorSimple.Direction.FORWARD);
        out1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "i");
        kick = hardwareMap.get(Servo.class, "k");
        wheel = hardwareMap.get(Servo.class, "pw");
        linear = hardwareMap.get(Servo.class, "li");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        bcs = hardwareMap.get(ColorSensor.class, "bottomColor");

        if (USE_WEBCAM)
            setManualExposure(1200, 5000);  // Use low exposure time to reduce motion blur

        selectStartingPosition();
        selectStartingDelay();
    }
}
