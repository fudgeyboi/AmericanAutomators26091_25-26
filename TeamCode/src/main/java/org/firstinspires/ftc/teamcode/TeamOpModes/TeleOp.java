package org.firstinspires.ftc.teamcode.TeamOpModes;

// Importing OpMode class
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Importing RoadRunner classes + dependencies
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFVelocityParams;

import org.firstinspires.ftc.teamcode.DriveException;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Vector;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    boolean prevdown = false;
    boolean prevup = false;
    int driveID = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        /// Set up the drive system
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initialPose);

        /// Main loop
        while (opModeIsActive()) {

            /// Update pose + reset if necessary
            mecanumDrive.updatePoseEstimate();

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
            } catch (Exception e) {
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
                } catch (Exception e2) {
                    throw new DriveException("Unhandled Exception in Drivetrain block.", e2);
                }
            }

            /// Code to control the drive system switching
            if (gamepad1.dpad_down && !prevdown && (driveID > 0)) {
                driveID -= 1;
            }
            if (gamepad1.dpad_up && !prevup && (driveID < 2)) {
                driveID += 1;
            }

            /// Set PrevState variables
            prevup = gamepad1.dpad_up;
            prevdown = gamepad1.dpad_down;

            /// Telemetry block
            telemetry.update();
        }
    }
}
