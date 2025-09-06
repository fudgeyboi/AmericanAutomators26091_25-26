package org.firstinspires.ftc.teamcode.TeamOpModes;

// Importing OpMode class
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Importing RoadRunner classes + dependencies
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFVelocityParams;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.teamcode.DriveException;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Vector;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    boolean prevdown = false;
    boolean prevup = false;
    boolean upPressed = false;
    boolean downPressed = false;
    double prevTime = 0;
    double deltaTime = 0;
    int driveID = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        /// Set up the drive system
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initialPose);
        TouchSensor touch = hardwareMap.get(TouchSensor.class, "touch");
        ElapsedTime runtime = new ElapsedTime();

        waitForStart();
        runtime.reset();
        /// Main loop
        while (opModeIsActive()) {

            /// Update pose + reset if necessary
            mecanumDrive.updatePoseEstimate();
            upPressed = gamepad1.dpad_up;
            downPressed = gamepad1.dpad_down;

            /// Try catch block
            try {
                /// Switch drive mode
                switch (driveID) {

                    case 1:

                        /// Field centric
                        double heading = mecanumDrive.localizer.getPose().heading.toDouble();
                        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                                new Vector2d(
                                        ((gamepad1.left_stick_y * java.lang.Math.sin(heading))
                                                - (gamepad1.left_stick_x * java.lang.Math.cos(heading))) / 1.5,
                                        ((gamepad1.left_stick_y * java.lang.Math.cos(heading))
                                                + (gamepad1.left_stick_x * java.lang.Math.sin(heading))) / 1.5
                                ),
                                -gamepad1.right_stick_x / 2
                        ));
                        telemetry.addData("DriveSystem is Field Centric", "");
                        break;

                    case 2:

                        /// Reserved for player centric
                        telemetry.addData("DriveSystem is Player Centric", "");
                        break;

                    default:

                        /// Robot centric
                        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                                new Vector2d(
                                        gamepad1.left_stick_x,
                                        gamepad1.left_stick_y
                                ),
                                gamepad1.right_stick_x
                        ));
                        telemetry.addData("DriveSystem is Robot Centric", "");
                        break;
                }
            } catch (RuntimeException e) {
                /// I couldn't explain this mess if I tried
                try {
                    mecanumDrive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    0,
                                    0
                            ),
                            0
                    ));
                    telemetry.addData("Handled Exception in Drivetrain block.", "");
                } catch (RuntimeException e2) {
                    throw new DriveException("Unhandled Exception in Drivetrain block.", e2);
                }
            }

            /// Code to control the drive system switching
            if (downPressed && !prevdown && (driveID > 0)) {
                driveID -= 1;
            } else if (downPressed && !prevdown) {
                driveID = 2;
            }
            if (upPressed && !prevup && (driveID < 2)) {
                driveID += 1;
            } else if (upPressed && !prevup) {
                driveID = 0;
            }

            /// Telemetry block
            deltaTime = runtime.milliseconds() - prevTime;
            prevTime = runtime.milliseconds();
            telemetry.addData("Tick time (ms)", deltaTime);
            telemetry.update();

            /// Set PrevState variables
            prevup = upPressed;
            prevdown = downPressed;
        }
    }
}
