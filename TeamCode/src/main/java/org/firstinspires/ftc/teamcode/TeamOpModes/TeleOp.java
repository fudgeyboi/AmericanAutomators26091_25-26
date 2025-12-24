package org.firstinspires.ftc.teamcode.TeamOpModes;


// Importing OpMode class
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Import hardware classes
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
    int tick = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        // Init
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initialPose);
        ElapsedTime runtime = new ElapsedTime();

        DcMotorEx launch = hardwareMap.get(DcMotorEx.class, "launch");
        launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launch.setDirection(DcMotorSimple.Direction.REVERSE);
        launch.setCurrentAlert(6, CurrentUnit.AMPS);
        launch.setVelocityPIDFCoefficients(1, 0.3, 0, 1.2);


        waitForStart();

        runtime.reset();

        // Main loop
        while (opModeIsActive() && !(gamepad1.back || gamepad2.back)) {

            TrajectoryActionBuilder returnTraj = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                    .strafeTo(new Vector2d(0, 0));

            Action returnToSender = returnTraj.build();

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

            // Actions
            {
                if (gamepad1.y) {
                    runningActions.add(returnToSender);
                }
            }

            // Update isPressed variables
            upPressed = gamepad1.dpad_up;
            downPressed = gamepad1.dpad_down;
            if (gamepad1.left_bumper) {
                tick--;
            }
            if (gamepad1.right_bumper) {
                tick++;
            }

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
                                                    - (gamepad1.left_stick_x * java.lang.Math.cos(heading))) / 1.5,
                                            ((gamepad1.left_stick_y * java.lang.Math.cos(heading))
                                                    + (gamepad1.left_stick_x * java.lang.Math.sin(heading))) / 1.5
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

            if (gamepad1.x) {
                launch.setVelocity(2050);
            } else if (gamepad1.b) {
                launch.setVelocity(1800);
            } else {
                launch.setVelocity(0);
            }

            // Telemetry block
            deltaTime = runtime.milliseconds() - prevTime;
            prevTime = runtime.milliseconds();
            packet.put("Tick time (ms)", deltaTime);
            dash.sendTelemetryPacket(packet);
            telemetry.addData("Speed (ticks/sec)", tick);
            telemetry.update();

            // Set PrevState variables
            prevup = upPressed;
            prevdown = downPressed;
        }
    }
}
