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

        Pose2d redSample1 = new Pose2d(-48, -17.5, Math.toRadians(-90));
        Pose2d redSample2 = new Pose2d(-58, -17.5, Math.toRadians(-90));
        Pose2d preGrab = new Pose2d(-32, -4, Math.toRadians(-180));
        Pose2d prePreGrab = new Pose2d(-32, -10, Math.toRadians(-180));
        Pose2d grab = new Pose2d(-32, 2, Math.toRadians(-180));
        switch(position){
            case Left:
                startPose = new Pose2d(41, 0, 0);
                endPose = new Pose2d(41, 0, 0);
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

        Trajectory toCenter = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0, 0, 0))
                .build();
        Trajectory toBar = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .strafeRight(30.5)
                .build();
        Trajectory toBarSpeed = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(0, -32), Math.toRadians(-90))
                .build();
        Trajectory awayFromBar = drive.trajectoryBuilder(new Pose2d(0, -31.5, 0))
                .strafeLeft(25)
                .build();
        Trajectory returnStart = drive.trajectoryBuilder(new Pose2d(0, -7, 0))
                .lineToLinearHeading(endPose)
                .build();
        Trajectory grabRed1 = drive.trajectoryBuilder(endPose)
                .lineToLinearHeading(redSample1)
                .build();
        Trajectory grabReturn1 = drive.trajectoryBuilder(redSample1)
                .lineToLinearHeading(preGrab)
                .build();
        Trajectory grabRed2 = drive.trajectoryBuilder(preGrab)
                .lineToLinearHeading(redSample2)
                .build();
        Trajectory grabReturn2 = drive.trajectoryBuilder(redSample2)
                .lineToLinearHeading(preGrab)
                .build();
        Trajectory clampPre = drive.trajectoryBuilder(redSample2)
                .lineToLinearHeading(new Pose2d(0, 0, 0))
                .build();
        Trajectory grabReturn3 = drive.trajectoryBuilder(new Pose2d(0, -7, 0))
                .lineToLinearHeading(prePreGrab)
                .build();
        Trajectory grabThatThang = drive.trajectoryBuilder(prePreGrab)
                .lineToLinearHeading(grab)
                .build();
        Trajectory toBar2 = drive.trajectoryBuilder(grab)
                .splineToSplineHeading(new Pose2d(-5, -31.5, Math.toRadians(0)), Math.toRadians(-90))
                .build();
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
        while(liftEncoder.getCurrentPosition() > -2675){
            lift1.setPower(1);
            lift2.setPower(1);
        }
        lift1.setPower(0);
        lift2.setPower(0);
        drive.followTrajectory(toBarSpeed);
        while(liftEncoder.getCurrentPosition() < -1500){
            lift1.setPower(-1);
            lift2.setPower(-1);
        }
        lift1.setPower(0);
        lift2.setPower(0);
        drive.followTrajectory(awayFromBar);
        drive.followTrajectory(returnStart);
        while(!down.isPressed()){
            lift1.setPower(-1);
            lift2.setPower(-1);
        }
        lift1.setPower(0);
        lift2.setPower(0);
        switch(position){
            case Left:
                break;
            case Right:
                wrist.setPosition(0.2);
                claw.setPower(1);
                arm.setPosition(0.16);
                drive.followTrajectory(grabRed1);
                arm.setPosition(0.210);
                sleep(450);
                claw.setPower(0);
                sleep(100);
                arm.setPosition(0.16);
                drive.followTrajectory(grabReturn1);
                claw.setPower(1);
                wrist.setPosition(0.2);
                arm.setPosition(0);
                sleep(250);
                arm.setPosition(0.16);
                drive.followTrajectory(grabRed2);
                arm.setPosition(0.210);
                sleep(450);
                claw.setPower(0);
                sleep(100);
                arm.setPosition(0.16);
                drive.followTrajectory(grabReturn2);
                claw.setPower(1);
                sleep(250);
                arm.setPosition(0);
                drive.followTrajectory(grabThatThang);
                //sleep(250);
                while(liftEncoder.getCurrentPosition() > -2675){
                    lift1.setPower(1);
                    lift2.setPower(1);
                }
                lift1.setPower(0);
                lift2.setPower(0);
                drive.followTrajectory(toBar2);
                /* while(liftEncoder.getCurrentPosition() > -2900){
                    lift1.setPower(1);
                    lift2.setPower(1);
                }
                lift1.setPower(0);
                lift2.setPower(0);
                drive.followTrajectory(toBar);
                 */
                while(liftEncoder.getCurrentPosition() < -1500){
                    lift1.setPower(-1);
                    lift2.setPower(-1);
                }
                lift1.setPower(0);
                lift2.setPower(0);
                drive.followTrajectory(awayFromBar);
                while(!down.isPressed()){
                    lift1.setPower(-1);
                    lift2.setPower(-1);
                }
                lift1.setPower(0);
                lift2.setPower(0);
                drive.followTrajectory(grabReturn3);
                claw.setPower(1);
                drive.followTrajectory(grabThatThang);
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
