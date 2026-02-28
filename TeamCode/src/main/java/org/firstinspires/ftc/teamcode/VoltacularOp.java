package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
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

@TeleOp(name="VoltacularOp", group="Linear OpMode")

public class VoltacularOp extends LinearOpMode {

    private ElapsedTime lastIntake = new ElapsedTime();

    // Nathaniel's play area

    final double DESIRED_DISTANCE = 12.0;

    final double SPEED_GAIN  =  0.02  ;
    final double STRAFE_GAIN =  0.015 ;
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

    private DcMotorEx out = null;
    private DcMotorEx out1 = null;
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
    private ColorSensor cs = null;
    private DigitalChannel laser = null;
    private Servo light1 = null;

    double  ws = 0;
    double test = 0;
    double s = 0;
    double sc = 0;
    private ColorSensor bcs = null;
    double[] size = {10, 1, 0.1, 0.01, 0.001};
    int index = 1;
    public float P = 35f;
    public float I = 0;
    public float D = 0.1f;
    public float F = 11.9f;
    double l = 0;
    double li = 0;

    public int[] PastOutputs = {0, 1, 2};

    public enum COLOR {
        GREEN,
        PURPLE,
        ANTINULL
    }
    public enum BALL {

        GREEN(COLOR.GREEN, 0),
        PURPLE(COLOR.PURPLE, 0),
        ANTINULL(COLOR.ANTINULL, 0);

        private COLOR ballColor;

        public final double[] INPUTPOSITIONS = {0.05,0.41,0.7672};
        public final double[] OUTPUTPOSITIONS = {0.595, 0.95, 0.24};

        BALL(COLOR ballColor, int ballPos) {
            this.ballColor = ballColor;
        }

        public COLOR getColor() {
            return ballColor;
        }

        public COLOR getInverseColor() {
            if(ballColor == COLOR.GREEN){
                return COLOR.PURPLE;
            } else if(ballColor == COLOR.PURPLE){
                return COLOR.GREEN;
            } else {
                return COLOR.ANTINULL;
            }
        }
    }
    public BALL[] Balls = {null,null,null};

    private BALL ProtoBall = BALL.GREEN;

    public int focusedBall = 0;

    public COLOR[] BallQueue = {null, null, null};

    public static COLOR[] colorSequence;

    public COLOR[] colorSequenceOnInit;

    public int shootStage = 0;

    public Boolean ShootToLoad = false;

    public double outPower = 1500;

    public int outToggle = 0;

    public int count = 0;
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
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        out = hardwareMap.get(DcMotorEx.class, "lr");
        out1 = hardwareMap.get(DcMotorEx.class, "ll");
        out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        out1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        out.setDirection(DcMotorSimple.Direction.FORWARD);
        out1.setDirection(DcMotorSimple.Direction.REVERSE);

        laser = hardwareMap.get(DigitalChannel.class, "laser");

        // flap = hardwareMap.get(Servo.class, "door");
        intake = hardwareMap.get(DcMotor.class, "i");
        kick = hardwareMap.get(Servo.class, "k");
        wheel = hardwareMap.get(Servo.class, "pw");
        linear = hardwareMap.get(Servo.class, "li");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        bcs = hardwareMap.get(ColorSensor.class, "bottomColor");
        light1 = hardwareMap.get(Servo.class, "Lrgb");





        //ashbaby
        if (USE_WEBCAM)
            setManualExposure(1200, 5000);  // Use low exposure time to reduce motion blur

        if(colorSequence != null){
            colorSequenceOnInit = colorSequence;
        } else {
            colorSequenceOnInit = new COLOR[]{COLOR.PURPLE, COLOR.PURPLE, COLOR.GREEN};
        }

        waitForStart();
        kick.setPosition(0.6);
        wheel.setPosition(ProtoBall.INPUTPOSITIONS[focusedBall]);
        lastIntake.reset();
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
                    if ((detection.id == 24) || (detection.id == 20)) {
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

                if(desiredTag.ftcPose.x != 6){
                    headingError = ((180 / Math.PI) * Math.atan(desiredTag.ftcPose.y / (desiredTag.ftcPose.x + 6)));
                }


                if(headingError < 0){
                    headingError = -(-90 - headingError);
                } else if(headingError > 0) {
                    headingError = -(90 - headingError);
                }



                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = -gamepad1.left_stick_y;
                turn   = Range.clip(Math.pow(headingError / EXPONENTIAL_TURN_RANGE, 3) * EXPONENTIAL_TURN_RANGE * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = -gamepad1.left_stick_x;

                telemetry.addData("April tag distance", desiredTag.ftcPose.y);
                telemetry.addData("changing angle",headingError);
            } else {

                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                drive  = -gamepad1.left_stick_y;
                strafe = -gamepad1.left_stick_x;
                turn   = -gamepad1.right_stick_x/2;
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            if(gamepad2.right_bumper && gamepad2.a){
                li = 1;
            } else {
                li = 0;
            }

            if(gamepad1.x){
                outPower = 1250;
                linear.setPosition(0.6);
            }
            if(gamepad1.y){
                outPower = 1400;
                linear.setPosition(0.525);
            }
            if(gamepad1.b){
                outPower = 1650;
                linear.setPosition(0.15);
            }
            if(gamepad2.dpad_up){
                l=l+0.05;
            }
            if(gamepad2.dpad_down){
                l=l-0.05;
            }

       /*     if(index<1){
                index = 5;
            }
            if(index>5){
                index = 1;
            }
            if(gamepad1.dpad_up){
                F = F+ size[index];
            }
            if(gamepad1.dpad_down){
                F = F- size[index];
            }
            if(gamepad1.dpad_right){
                P = P+ size[index];
            }if(gamepad1.dpad_left){
                P = P- size[index];
            }
            if(gamepad1.right_bumper){
                index=index+1;
            }
            if(gamepad1.left_bumper){
                index=index-1;
            }


*/ //15.5, 17, 27.89
            if(Balls == null){
                light1.setPosition(0);
            } else{
                int c = 0;
                for (Object element : Balls) {
                    if (element != null) {
                        c++;
                        count = c;
                    }
                }
            }
            if(count == 1){
            light1.setPosition(0.277);
            }else if(count == 2){
                light1.setPosition(0.388);
            }else if(count == 3){
                light1.setPosition(0.444);
            }

            out1.setVelocity(outPower * outToggle);
            double velocity = out1.getVelocity();
            double error = outPower-out1.getVelocity();
            out.setVelocity(outPower * outToggle);
            PIDFCoefficients test2 = new PIDFCoefficients(P, I, D, F);
            out.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,test2);
            PIDFCoefficients test1 = new PIDFCoefficients(P, I, D, F);
            out1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,test1);
            telemetry.addData("velocity", velocity);
            telemetry.addData("error", error);
            telemetry.addData("index", index);
            telemetry.addData("P", P);
            telemetry.addData("F", F);
            // P = 19.2, F = 21.2
            if(gamepad1.a){
                ws=-0.15;
            }else if (gamepad1.b){
                ws = 0;
            }
            telemetry.addData("Power", power);
            /*
            if(gamepad2.left_bumper){
                if(test<0.5){
                    kick.setPosition(0.15);
                    sleep(250);
                }
                if(test>0.5){
                    kick.setPosition(0.6);
                    runtime.reset();
                }
            }else{
                kick.setPosition(0.15);
                if(runtime.seconds() > 0.25) {
                    if (gamepad2.x) {
                        wheel.setPosition(0);
                        test = 0;
                    }
                    if (gamepad2.y) {
                        wheel.setPosition(0.7272);
                        test = 0;
                    }
                    if (gamepad2.b) {
                        wheel.setPosition(0.354);
                        test = 0;
                    }
                    if (gamepad2.x && gamepad2.right_bumper) {
                        wheel.setPosition(0.535);
                        test = 1;
                        sleep(250);
                    }
                    if (gamepad2.y && gamepad2.right_bumper) {
                        wheel.setPosition(0.162);
                        test = 1;
                        sleep(250);
                    }
                    if (gamepad2.b && gamepad2.right_bumper) {
                        wheel.setPosition(0.908);
                        test = 1;
                        sleep(250);
                    }
                }
            }

             */
            telemetry.addData("TestInput", false);


            if(gamepad2.left_bumper && (Balls[0] == null || Balls[1] == null || Balls[2] == null) && shootStage == 0) {
                BallQueue = new COLOR[] {null, null, null};
                telemetry.addData("TestInput", true);
                bcs.enableLed(true);
                intake.setPower(1);
                wheel.setPosition(ProtoBall.INPUTPOSITIONS[focusedBall]);
                if(!ShootToLoad || lastIntake.milliseconds() > 1250){
                    ShootToLoad = false;
                    if (laser.getState() && lastIntake.milliseconds() > 600) {
                        telemetry.addData("FinalInput", true);
                        lastIntake.reset();
                        if ((bcs.red() + bcs.blue()) / 2.0 > bcs.green()) {
                            Balls[focusedBall] = BALL.PURPLE;
                        } else {
                            Balls[focusedBall] = BALL.GREEN;
                        }
                        for (int i = 0; i < Balls.length; i++) {
                            if (Balls[i] == null) {
                                focusedBall = i;
                                break;
                            }
                        }
                    }
                }
            } else {
                if(shootStage == 0){
                    if(BallQueue[0] == null && BallQueue[1] == null && BallQueue[2] == null) {
                        if (gamepad2.y) {
                            BallQueue = new COLOR[]{COLOR.GREEN, null, null};
                        } else if (gamepad2.x) {
                            BallQueue = new COLOR[]{COLOR.PURPLE, null, null};
                        } else if (gamepad2.b) {
                            BallQueue[0] = colorSequenceOnInit[0];
                            BallQueue[1] = colorSequenceOnInit[1];
                            BallQueue[2] = colorSequenceOnInit[2];
                        } else if (gamepad2.a) {
                            Balls[PastOutputs[2]] = BALL.ANTINULL;
                            BallQueue = new COLOR[]{COLOR.ANTINULL, null, null};
                        }
                    } else if(BallQueue[0] != null){
                        boolean foundBall = false;
                        for (int i = 0; i < Balls.length; i++) {
                            if (Balls[i] != null && Balls[i].getColor() == BallQueue[0]) {
                                focusedBall = i;
                                foundBall = true;
                                break;
                            }
                        }
                        if(!foundBall){
                            for (int i = 0; i < Balls.length; i++) {
                                if (Balls[i] != null) {
                                    focusedBall = i;
                                    foundBall = true;
                                    break;
                                }
                            }
                        }


                        if(foundBall){
                            outToggle = 1;
                            shootStage = 1;

                        } else {
                            BallQueue = new COLOR[] {null, null, null};
                        }
                    }
                    lastIntake.reset();
                } else if(shootStage == 1 && lastIntake.milliseconds() > 500){
                    kick.setPosition(0.15);
                    shootStage = 2;
                    lastIntake.reset();
                } else if(shootStage == 2 && lastIntake.milliseconds() > 400){
                    kick.setPosition(0.6);
                    shootStage = 3;
                    lastIntake.reset();
                } else if(shootStage == 3 && lastIntake.milliseconds() > 350){
                    outToggle = 0;
                    Balls[focusedBall] = null;
                    BallQueue[0] = BallQueue[1];
                    BallQueue[1] = BallQueue[2];
                    BallQueue[2] = null;
                    shootStage = 0;
                    int tempCount = 0;
                    int[] temp = {focusedBall, 0, 0};
                    for(int i = 0; i < PastOutputs.length; i++){
                        if(PastOutputs[i] != focusedBall){
                            tempCount++;
                            temp[tempCount] = PastOutputs[i];
                        }
                    }
                    PastOutputs[0] = temp[0];
                    PastOutputs[1] = temp[1];
                    PastOutputs[2] = temp[2];
                    lastIntake.reset();
                }


                ShootToLoad = true;
                intake.setPower(0);
                wheel.setPosition(ProtoBall.OUTPUTPOSITIONS[focusedBall]);
                bcs.enableLed(false);
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
            if(gamepad1.right_bumper){
                sc=0.5;
            }else{
                sc=1;
            }



            moveRobot(drive, strafe, turn);
            telemetry.addData("BallPastQueue", PastOutputs[0] + ", " + PastOutputs[1] + ", " + PastOutputs[2]);
            telemetry.addData("test", test);
            telemetry.addData("out",out.getPower());
            telemetry.addData("kick",kick.getPosition());
            telemetry.update();

            //telemetry.update();
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
        leftFrontDrive.setPower(leftFrontPower*sc);
        rightFrontDrive.setPower(rightFrontPower*sc);
        leftBackDrive.setPower(leftBackPower*sc);
        rightBackDrive.setPower(rightBackPower*sc);
        telemetry.addData("LastIntake", lastIntake.milliseconds());
        telemetry.addData("ShootStage", shootStage);
        telemetry.addData("BallQueue", BallQueue[0] + ", "+ BallQueue[1] + ", "+ BallQueue[2]);
        telemetry.addData("StoredSequence", colorSequenceOnInit[0] + ", "+ colorSequenceOnInit[1] + ", "+ colorSequenceOnInit[2]);
        if(Balls[0] != null)
            telemetry.addData("1", Balls[0].getColor());
        if(Balls[1] != null)
            telemetry.addData("2", Balls[1].getColor());
        if(Balls[2] != null)
            telemetry.addData("3", Balls[2].getColor());
        telemetry.addData("Color sensor value fixed green", bcs.green() / ((double)bcs.alpha()));
        telemetry.addData("Color sensor value fixed green", ((bcs.red() + bcs.blue()) / 2.0) / ((double)bcs.alpha()));
        telemetry.update();
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