package org.firstinspires.ftc.teamcode.TeamOpModes;


// Importing OpMode class
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Import hardware classes
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

// Import RoadRunner classes + dependencies
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

// Import custom classes
import org.firstinspires.ftc.teamcode.DriveException;

import java.util.ArrayList;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    boolean prevdown = false;
    boolean prevup = false;
    boolean upPressed = false;
    boolean downPressed = false;
    double prevTime = 0;
    double deltaTime = 0;
    int driveID = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize RoadRunner
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(-90));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initialPose);

        // Create stopwatch
        ElapsedTime runtime = new ElapsedTime();

        // Initialize hardware
        DcMotorEx launch = hardwareMap.get(DcMotorEx.class, "launch");
        launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launch.setCurrentAlert(8, CurrentUnit.AMPS);
        launch.setVelocityPIDFCoefficients(16, 1, 0.1, 2);
        DcMotor intake = hardwareMap.get(DcMotorEx.class, "intake");
        Servo flip = hardwareMap.get(Servo.class, "flip");
        flip.setDirection(Servo.Direction.REVERSE);
        CRServo spindexer = hardwareMap.get(CRServo.class, "spindexer");

        // Init limbo
        waitForStart();

        // Reset stopwatch
        runtime.reset();

        // Main loop
        while (opModeIsActive() && !(gamepad1.back || gamepad2.back)) {

            // Create dashboard packet
            TelemetryPacket packet = new TelemetryPacket();

            // Update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            // Update pose + reset if necessary
            mecanumDrive.updatePoseEstimate();

            // Queue actions
            {
                // Blue recenter
                if (gamepad1.x && runningActions.isEmpty()) {
                    TrajectoryActionBuilder leftClose = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                            .strafeToSplineHeading(new Vector2d(0, 0), Math.toRadians(135));
                    Action aimLeftClose = leftClose.build();
                    runningActions.add(aimLeftClose);
                }

                // Red recenter
                if (gamepad1.b && runningActions.isEmpty()) {
                    TrajectoryActionBuilder rightClose = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                            .strafeToSplineHeading(new Vector2d(0, 0), Math.toRadians(45));
                    Action aimRightClose = rightClose.build();
                    runningActions.add(aimRightClose);
                }
            }

            // Update isPressed variables
            upPressed = gamepad1.dpad_up;
            downPressed = gamepad1.dpad_down;

            // Try catch block
            try {
                // Switch drive mode
                if (runningActions.isEmpty()) {
                    switch (driveID) {

                        case 1:

                            // Field centric
                            double heading = mecanumDrive.localizer.getPose().heading.toDouble();
                            mecanumDrive.setDrivePowers(new PoseVelocity2d(
                                    new Vector2d(
                                            ((gamepad1.left_stick_y * java.lang.Math.sin(heading))
                                                    - (gamepad1.left_stick_x * java.lang.Math.cos(heading))),
                                            ((gamepad1.left_stick_y * java.lang.Math.cos(heading))
                                                    + (gamepad1.left_stick_x * java.lang.Math.sin(heading)))
                                    ),
                                    -gamepad1.right_stick_x
                            ));
                            packet.put("DriveSystem is", "Field Centric");
                            break;

                        case 2:

                            // Reserved for player centric
                            packet.put("DriveSystem is", "Player Centric");
                            break;

                        case 3:
                            mecanumDrive.setDrivePowers(new PoseVelocity2d(
                                    new Vector2d(
                                            -gamepad1.left_stick_y,
                                            0
                                    ),
                                    gamepad1.right_stick_x
                            ));
                            packet.put("DriveSystem is", "HammerHead Mode");
                        default:

                            // Robot centric
                            mecanumDrive.setDrivePowers(new PoseVelocity2d(
                                    new Vector2d(
                                            -gamepad1.left_stick_y,
                                            -gamepad1.left_stick_x
                                    ),
                                    -gamepad1.right_stick_x
                            ));
                            packet.put("DriveSystem is", "Robot Centric");
                            break;
                    }
                }
            } catch (RuntimeException e) {
                // I couldn't explain this mess if I tried
                try {
                    mecanumDrive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    0,
                                    0
                            ),
                            0
                    ));
                    packet.put("Handled Exception in Drivetrain block.", "");
                } catch (RuntimeException e2) {
                    throw new DriveException("Unhandled Exception in Drivetrain block.", e2);
                }
            }

            // Code to control the drive system switching
            if (downPressed && !prevdown && (driveID > 0)) {
                driveID -= 1;
            } else if (downPressed && !prevdown) {
                driveID = 3;
            }
            if (upPressed && !prevup && (driveID < 3)) {
                driveID += 1;
            } else if (upPressed && !prevup) {
                driveID = 0;
            }

            // Misc motors block
            {
                if (gamepad2.x) {
                    launch.setVelocity(2600);
                } else if (gamepad2.b) {
                    launch.setVelocity(2350);
                } else {
                    launch.setVelocity(0);
                }
            }

            // Servo block
            {
                if (gamepad2.y) {
                    flip.setPosition(0.52);
                } else {
                    flip.setPosition(0.1);
                }
                spindexer.setPower(gamepad2.left_stick_x / 5);
            }

            intake.setPower(gamepad2.right_trigger);

            // Add data to telemetry
            deltaTime = runtime.milliseconds() - prevTime;
            prevTime = runtime.milliseconds();
            packet.put("Tick time (ms)", deltaTime);
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), mecanumDrive.localizer.getPose());
            telemetry.addData("Velocity (ticks/sec)", launch.getVelocity());

            // Send telemetry
            dash.sendTelemetryPacket(packet);
            telemetry.update();

            // Set PrevState variables
            prevup = upPressed;
            prevdown = downPressed;
        }
    }
}
