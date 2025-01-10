/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous(name="Voltanomous", group="Autonomous", preselectTeleOp = "VoltaTeleOp")

public class Voltanomous extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private TouchSensor up, down;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo arm = null;
    private CRServo claw = null;
    private Servo wrist = null;
    private Servo extend = null;
    private DcMotor lift1 = null;
    private DcMotor lift2 = null;
    private Encoder liftEncoder;
    private Servo hockey = null;

    private long delay = 0;
    public enum START_POSITION{
        Left,
        Right
    }

    public START_POSITION position;

    public void initVars(){
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "flw");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "blw");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frw");
        rightBackDrive = hardwareMap.get(DcMotor.class, "brw");
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(CRServo.class, "c");
        wrist = hardwareMap.get(Servo.class, "w");
        lift1 = hardwareMap.get(DcMotor.class, "l");
        lift2 = hardwareMap.get(DcMotor.class, "l2");
        extend = hardwareMap.get(Servo.class, "e");
        up = hardwareMap.get(TouchSensor.class, "UpS");
        down = hardwareMap.get(TouchSensor.class, "DoS");
        hockey = hardwareMap.get(Servo.class, "hockey");
        hockey.setPosition(0);

        liftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "l2"));

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setDirection(DcMotor.Direction.REVERSE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        position = START_POSITION.Right;
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
                    telemetry.addData("Remove your hand from the bumper", "");
                    telemetry.update();
                }
            }
            if (gamepad1.right_bumper) {
                position = START_POSITION.Right;
                while (gamepad1.dpad_down && !isStopRequested()) {
                    telemetry.addData("Remove your hand from the bumper", "");
                    telemetry.update();
                }
            }
            telemetry.update();
        }
    }

    public void selectStartingDelay() {
        while (gamepad1.a && !isStopRequested()) {
            telemetry.addData("Remove your hand from the a button", "");
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
                    telemetry.addData("Remove your hand from the dpad", "");
                    telemetry.update();
                }
            }
            if (gamepad1.dpad_down) {
                delay -= 500;
                while (gamepad1.dpad_down && !isStopRequested()) {
                    telemetry.addData("Remove your hand from the dpad", "");
                    telemetry.update();
                }
            }
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() {
        initVars();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        selectStartingPosition();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);
        Pose2d endPose = new Pose2d(0, 0, 0);

        Pose2d colorSample1 = new Pose2d(-40, -40, Math.toRadians(-90));
        Pose2d colorSample2 = new Pose2d(-49, -40, Math.toRadians(-90));
        Pose2d colorSample3 = new Pose2d(-58, -40, Math.toRadians(-90));
        Pose2d yellowSample1 = new Pose2d(58, -21, Math.toRadians(-90));
        Pose2d yellowSample2 = new Pose2d(68.5, -21, Math.toRadians(-90));
        Pose2d yellowSample3 = new Pose2d(57, -35, Math.toRadians(0));
        Pose2d preDrop = new Pose2d(56, -9, Math.toRadians(45));
        Pose2d drop = new Pose2d(66, -3, Math.toRadians(45));
        Pose2d hook1 = new Pose2d(-14, -33, Math.toRadians(0));
        Pose2d hook2 = new Pose2d(-10, -33, Math.toRadians(-360));
        Pose2d hook3 = new Pose2d(-6, -33, Math.toRadians(-360));
        Pose2d hook4 = new Pose2d(-2, -32, Math.toRadians(-360));
        Pose2d dropGrabbed1 = new Pose2d(-40, 0, Math.toRadians(-90));
        Pose2d dropGrabbed2 = new Pose2d(-49, 0, Math.toRadians(-90));
        Pose2d dropGrabbed3 = new Pose2d(-58, 0, Math.toRadians(-90));
        Pose2d prePreGrab = new Pose2d(-31, -10, Math.toRadians(-180));
        Pose2d grab = new Pose2d(-31, 2, Math.toRadians(-180));
        switch(position){
            case Left:
                startPose = new Pose2d(41, 0, 0);
                endPose = new Pose2d(32, -57, Math.toRadians(-90));
                claw.setPower(1);
                arm.setPosition(1);
                break;
            case Right:
                startPose = new Pose2d(-15, 0, 0);
                endPose = new Pose2d(-50, -5, 0);
                break;
        }

        ElapsedTime timer = new ElapsedTime();
        telemetry.addData("Start pose", startPose);
        telemetry.update();

        drive.setPoseEstimate(startPose);
        //left side
        Trajectory toDrop1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(preDrop)
                .build();
        Trajectory Drop1 = drive.trajectoryBuilder(toDrop1.end())
                .lineToLinearHeading(drop)
                .build();
        Trajectory antiDrop1 = drive.trajectoryBuilder(Drop1.end())
                .lineToLinearHeading(preDrop)
                .build();
        Trajectory toGrab1 = drive.trajectoryBuilder(antiDrop1.end())
                .lineToLinearHeading(yellowSample1)
                .build();
        Trajectory toDrop2 = drive.trajectoryBuilder(toGrab1.end())
                .lineToLinearHeading(preDrop)
                .build();
        Trajectory Drop2 = drive.trajectoryBuilder(toDrop2.end())
                .lineToLinearHeading(drop)
                .build();
        Trajectory antiDrop2 = drive.trajectoryBuilder(Drop2.end())
                .lineToLinearHeading(preDrop)
                .build();
        Trajectory toGrab2 = drive.trajectoryBuilder(antiDrop2.end())
                .lineToLinearHeading(yellowSample2)
                .build();
        Trajectory toDrop3 = drive.trajectoryBuilder(toGrab2.end())
                .lineToLinearHeading(preDrop)
                .build();
        Trajectory Drop3 = drive.trajectoryBuilder(toDrop3.end())
                .lineToLinearHeading(drop)
                .build();
        Trajectory antiDrop3 = drive.trajectoryBuilder(Drop3.end())
                .lineToLinearHeading(preDrop)
                .build();
        Trajectory toGrab3 = drive.trajectoryBuilder(antiDrop3.end())
                .lineToLinearHeading(yellowSample3)
                .build();
        Trajectory toDrop4 = drive.trajectoryBuilder(toGrab3.end())
                .lineToLinearHeading(preDrop)
                .build();
        Trajectory Drop4 = drive.trajectoryBuilder(toDrop4.end())
                .lineToLinearHeading(drop)
                .build();
        Trajectory antiDrop4 = drive.trajectoryBuilder(Drop4.end())
                .lineToLinearHeading(preDrop)
                .build();
        Trajectory parkLeft = drive.trajectoryBuilder(antiDrop4.end())
                .splineToLinearHeading(endPose, Math.toRadians(90))
                .build();
        //right side
        Trajectory toBarSpeed = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(hook1, Math.toRadians(-90))
                .build();
        Trajectory awayFromBar = drive.trajectoryBuilder(toBarSpeed.end())
                .strafeLeft(25)
                .build();
        Trajectory grabRed1 = drive.trajectoryBuilder(awayFromBar.end())
                .lineToLinearHeading(colorSample1)
                .build();
        TrajectorySequence grabSamples = drive.trajectorySequenceBuilder(grabRed1.end())
                .lineToLinearHeading(dropGrabbed1)
                .forward(40)
                .lineToLinearHeading(colorSample2)
                .lineToLinearHeading(dropGrabbed2)
                .forward(40)
                .lineToLinearHeading(colorSample3)
                .lineToLinearHeading(dropGrabbed3)
                .build();
        TrajectorySequence grabSamples1 = drive.trajectorySequenceBuilder(grabSamples.end())
                .lineToLinearHeading(prePreGrab)
                .lineToLinearHeading(grab)
                .build();

        Trajectory toBar1 = drive.trajectoryBuilder(grabSamples1.end())
                .splineToLinearHeading(hook2, Math.toRadians(-90))
                .build();
        Trajectory preGrab1 = drive.trajectoryBuilder(toBar1.end())
                .lineToLinearHeading(prePreGrab)
                .build();
        Trajectory grab1 = drive.trajectoryBuilder(preGrab1.end())
                .lineToLinearHeading(grab)
                .build();
        Trajectory toBar2 = drive.trajectoryBuilder(grab1.end())
                .splineToLinearHeading(hook3, Math.toRadians(-90))
                .build();
        Trajectory preGrab2 = drive.trajectoryBuilder(toBar2.end())
                .lineToLinearHeading(prePreGrab)
                .build();
        Trajectory grab2 = drive.trajectoryBuilder(preGrab2.end())
                .lineToLinearHeading(grab)
                .build();
        Trajectory toBar3 = drive.trajectoryBuilder(grab2.end())
                .splineToLinearHeading(hook4, Math.toRadians(-90))
                .build();
        /*
        Trajectory switchGrab = drive.trajectoryBuilder(dropGrabbed)
                .lineToLinearHeading(prePreGrab)
                .build();
        Trajectory clampPre = drive.trajectoryBuilder(colorSample2)
                .lineToLinearHeading(new Pose2d(0, 0, 0))
                .build();
        Trajectory grabReturn3 = drive.trajectoryBuilder(new Pose2d(-9, -7, 0))
                .lineToLinearHeading(prePreGrab)
                .build();
        Trajectory grabThatThang = drive.trajectoryBuilder(prePreGrab)
                .lineToLinearHeading(grab)
                .build();
        Trajectory toBar2 = drive.trajectoryBuilder(grab)
                .splineToSplineHeading(new Pose2d(-9, -31.5, Math.toRadians(0)), Math.toRadians(-90))
                .build();
        Trajectory toBar3 = drive.trajectoryBuilder(grab)
                .splineToSplineHeading(new Pose2d(-11.5, -31.5, Math.toRadians(0)), Math.toRadians(-90))
                .build();
         */
        selectStartingDelay();

        telemetry.clearAll();
        telemetry.addData("Position Set to  ", position);
        telemetry.addData("Delay Set to  ", delay);
        telemetry.addData("All Ready.", "");
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;

        sleep(delay);

        //drive.followTrajectory(toCenter);

        switch(position){
            case Left:
                claw.setPower(1);
                wrist.setPosition(0.2);
                lift1.setPower(1);
                lift2.setPower(1);
                drive.followTrajectoryAsync(toDrop1);
                while(!up.isPressed() || drive.isBusy()){
                    if(drive.isBusy()){
                        drive.update();
                    }
                    if(!up.isPressed()){
                        lift1.setPower(1);
                        lift2.setPower(1);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }
                lift1.setPower(0);
                lift2.setPower(0);
                drive.followTrajectory(Drop1);
                claw.setPower(0);
                sleep(400);
                drive.followTrajectory(antiDrop1);
                arm.setPosition(0);
                wrist.setPosition(0.8);
                lift1.setPower(-1);
                lift2.setPower(-1);
                drive.followTrajectoryAsync(toGrab1);
                while(!down.isPressed() || drive.isBusy()){
                    if(drive.isBusy()){
                        drive.update();
                    }
                    if(!down.isPressed()){
                        lift1.setPower(-1);
                        lift2.setPower(-1);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }
                lift1.setPower(0);
                lift2.setPower(0);
                wrist.setPosition(0.8);
                arm.setPosition(0);
                sleep(400);
                claw.setPower(1);
                sleep(100);
                arm.setPosition(1);
                wrist.setPosition(0.2);
                lift1.setPower(1);
                lift2.setPower(1);
                drive.followTrajectoryAsync(toDrop2);
                while(!up.isPressed() || drive.isBusy()){
                    if(drive.isBusy()){
                        drive.update();
                    }
                    if(!up.isPressed()){
                        lift1.setPower(1);
                        lift2.setPower(1);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }
                lift1.setPower(0);
                lift2.setPower(0);
                drive.followTrajectory(Drop2);
                claw.setPower(0);
                sleep(400);
                drive.followTrajectory(antiDrop2);
                arm.setPosition(0);
                wrist.setPosition(0.8);
                lift1.setPower(-1);
                lift2.setPower(-1);
                drive.followTrajectoryAsync(toGrab2);
                while(!down.isPressed() || drive.isBusy()){
                    if(drive.isBusy()){
                        drive.update();
                    }
                    if(!down.isPressed()){
                        lift1.setPower(-1);
                        lift2.setPower(-1);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }
                lift1.setPower(0);
                lift2.setPower(0);
                wrist.setPosition(0.8);
                arm.setPosition(0);
                sleep(400);
                claw.setPower(1);
                sleep(100);
                arm.setPosition(1);
                wrist.setPosition(0.2);
                lift1.setPower(1);
                lift2.setPower(1);
                drive.followTrajectoryAsync(toDrop3);
                while(!up.isPressed() || drive.isBusy()){
                    if(drive.isBusy()){
                        drive.update();
                    }
                    if(!up.isPressed()){
                        lift1.setPower(1);
                        lift2.setPower(1);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }
                lift1.setPower(0);
                lift2.setPower(0);
                drive.followTrajectory(Drop3);
                claw.setPower(0);
                sleep(400);
                drive.followTrajectory(antiDrop3);
                arm.setPosition(0);
                wrist.setPosition(0.2);
                lift1.setPower(-1);
                lift2.setPower(-1);
                drive.followTrajectoryAsync(toGrab3);
                while(!down.isPressed() || drive.isBusy()){
                    if(drive.isBusy()){
                        drive.update();
                    }
                    if(!down.isPressed()){
                        lift1.setPower(-1);
                        lift2.setPower(-1);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }
                lift1.setPower(0);
                lift2.setPower(0);
                wrist.setPosition(0.2);
                arm.setPosition(0);
                sleep(400);
                claw.setPower(1);
                sleep(100);
                arm.setPosition(1);
                wrist.setPosition(0.2);
                lift1.setPower(1);
                lift2.setPower(1);
                drive.followTrajectoryAsync(toDrop4);
                while(!up.isPressed() || drive.isBusy()){
                    if(drive.isBusy()){
                        drive.update();
                    }
                    if(!up.isPressed()){
                        lift1.setPower(1);
                        lift2.setPower(1);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }
                lift1.setPower(0);
                lift2.setPower(0);
                drive.followTrajectory(Drop4);
                claw.setPower(0);
                sleep(400);
                drive.followTrajectory(antiDrop4);
                lift1.setPower(-1);
                lift2.setPower(-1);
                drive.followTrajectoryAsync(parkLeft);
                while(liftEncoder.getCurrentPosition() < -1850 || drive.isBusy()){
                    if(drive.isBusy()){
                        drive.update();
                    }
                    if(liftEncoder.getCurrentPosition() < -1850){
                        lift1.setPower(-1);
                        lift2.setPower(-1);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }
                lift1.setPower(0);
                lift2.setPower(0);
                break;
            case Right:
                drive.followTrajectoryAsync(toBarSpeed);
                while(liftEncoder.getCurrentPosition() > -2600 || drive.isBusy()){
                    if(drive.isBusy()){
                        drive.update();
                    }
                    if(liftEncoder.getCurrentPosition() > -2600){
                        lift1.setPower(1);
                        lift2.setPower(1);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }
                lift1.setPower(0);
                lift2.setPower(0);
                while(liftEncoder.getCurrentPosition() < -1500){
                    lift1.setPower(-1);
                    lift2.setPower(-1);
                }
                lift1.setPower(-1);
                lift2.setPower(-1);
                drive.followTrajectoryAsync(awayFromBar);
                while(!down.isPressed() || drive.isBusy()){
                    if(drive.isBusy()){
                        drive.update();
                    }
                    if(!down.isPressed()){
                        lift1.setPower(-1);
                        lift2.setPower(-1);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }
                lift1.setPower(0);
                lift2.setPower(0);
                drive.followTrajectory(grabRed1);
                hockey.setPosition(1);
                sleep(300);
                drive.followTrajectorySequence(grabSamples);
                hockey.setPosition(0);
                drive.followTrajectorySequence(grabSamples1);
                drive.followTrajectoryAsync(toBar1);
                while(liftEncoder.getCurrentPosition() > -2600  || drive.isBusy()){
                    if(drive.isBusy()){
                        drive.update();
                    }
                    if(liftEncoder.getCurrentPosition() > -2600){
                        lift1.setPower(1);
                        lift2.setPower(1);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }
                lift1.setPower(0);
                lift2.setPower(0);
                while(liftEncoder.getCurrentPosition() < -1500){
                    lift1.setPower(-1);
                    lift2.setPower(-1);
                }
                lift1.setPower(-1);
                lift2.setPower(-1);
                drive.followTrajectory(preGrab1);
                drive.followTrajectory(grab1);
                drive.followTrajectoryAsync(toBar2);
                while(liftEncoder.getCurrentPosition() > -2600 || drive.isBusy()){
                    if(drive.isBusy()){
                        drive.update();
                    }
                    if(liftEncoder.getCurrentPosition() > -2600){
                        lift1.setPower(1);
                        lift2.setPower(1);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }
                lift1.setPower(0);
                lift2.setPower(0);
                while(liftEncoder.getCurrentPosition() < -1500){
                    lift1.setPower(-1);
                    lift2.setPower(-1);
                }
                lift1.setPower(-1);
                lift2.setPower(-1);
                drive.followTrajectoryAsync(preGrab2);
                while((!down.isPressed() || drive.isBusy()) && opModeIsActive()){
                    if(drive.isBusy()){
                        drive.update();
                    }
                    if(!down.isPressed()){
                        lift1.setPower(-1);
                        lift2.setPower(-1);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }
                /*
                claw.setPower(1);
                sleep(250);
                arm.setPosition(1);
                drive.followTrajectory(switchGrab);
                drive.followTrajectory(grabThatThang);
                //sleep(250);
                while(liftEncoder.getCurrentPosition() > -500){
                    lift1.setPower(1);
                    lift2.setPower(1);
                }
                lift1.setPower(1);
                lift2.setPower(1);
                drive.followTrajectoryAsync(toBar2);
                while(liftEncoder.getCurrentPosition() > -2600 || drive.isBusy()){
                    if(drive.isBusy()){
                        drive.update();
                    }
                    if(liftEncoder.getCurrentPosition() > -2600){
                        lift1.setPower(1);
                        lift2.setPower(1);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }
                while(liftEncoder.getCurrentPosition() < -1500){
                    lift1.setPower(-1);
                    lift2.setPower(-1);
                }
                lift1.setPower(0);
                lift2.setPower(0);
                drive.followTrajectoryAsync(awayFromBar);
                while(drive.isBusy()){
                    if(drive.isBusy()){
                        drive.update();
                    }
                    if(!down.isPressed()){
                        lift1.setPower(-1);
                        lift2.setPower(-1);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }
                drive.followTrajectoryAsync(grabReturn3);
                while(!down.isPressed() || drive.isBusy()){
                    if(drive.isBusy()){
                        drive.update();
                    }
                    if(!down.isPressed()){
                        lift1.setPower(-1);
                        lift2.setPower(-1);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }
                lift1.setPower(0);
                lift2.setPower(0);
                claw.setPower(1);
                drive.followTrajectory(grabThatThang);
                while(liftEncoder.getCurrentPosition() > -500){
                    lift1.setPower(1);
                    lift2.setPower(1);
                }
                lift1.setPower(1);
                lift2.setPower(1);
                drive.followTrajectoryAsync(toBar3);
                while(liftEncoder.getCurrentPosition() > -2600 || drive.isBusy()){
                    if(drive.isBusy()){
                        drive.update();
                    }
                    if(liftEncoder.getCurrentPosition() > -2600){
                        lift1.setPower(1);
                        lift2.setPower(1);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }
                while(liftEncoder.getCurrentPosition() < -1500){
                    lift1.setPower(-1);
                    lift2.setPower(-1);
                }
                lift1.setPower(0);
                lift2.setPower(0);
                drive.followTrajectoryAsync(awayFromBar);
                while(drive.isBusy()){
                    if(drive.isBusy()){
                        drive.update();
                    }
                    if(!down.isPressed()){
                        lift1.setPower(-1);
                        lift2.setPower(-1);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }
                drive.followTrajectoryAsync(grabReturn3);
                while(!down.isPressed() || drive.isBusy()){
                    if(drive.isBusy()){
                        drive.update();
                    }
                    if(!down.isPressed()){
                        lift1.setPower(-1);
                        lift2.setPower(-1);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }
                */
                break;
        }
        runtime.reset();
        float wr = 0.8f;
        waitForStart();
        while (opModeIsActive()) {

            wr = Math.max(wr,0.2f);
            wr = Math.min(wr,0.8f);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Up",up.isPressed());
            telemetry.addData("liftEncoder", liftEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
