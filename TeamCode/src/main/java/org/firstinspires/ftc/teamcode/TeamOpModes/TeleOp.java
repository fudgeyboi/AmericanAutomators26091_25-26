package org.firstinspires.ftc.teamcode.TeamOpModes;


// Importing OpMode class
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Import hardware classes
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

// Import computing libraries
import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

// Import RoadRunner classes + dependencies
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drawing;


// Import custom classes
import org.firstinspires.ftc.teamcode.DriveException;
import org.firstinspires.ftc.teamcode.TeamOpModes.ActionConfig.*;


// Import NextFTC classes
import static dev.nextftc.bindings.Bindings.*;
import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    // Declare and initialize global variables
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private List<Action> trajectoryActions = new ArrayList<>();
    double prevTime = 0;
    double deltaTime = 0;
    int driveID = 3;
    String closeOrFar = "close";
    int playerX = 0;
    int playerY = 0;

    // FINALLY GOT PLAYER CENTRIC WORKING! WOOHOO!
    Vector2d PCDrivePowers(Pose2d pose, double gamepadx, double gamepady) {

        double heading = pose.heading.toDouble();
        double x = pose.position.x;
        double y = pose.position.y;

        double theta = Math.atan2(x - playerX, y - playerY);

        Vector2d powerVec = new Vector2d(
                ((gamepady * java.lang.Math.sin(theta + heading))
                        - (gamepadx * java.lang.Math.cos(theta + heading))),
                ((gamepady * java.lang.Math.cos(theta + heading))
                        + (gamepadx * java.lang.Math.sin(theta + heading)))
        );

        powerVec.norm();

        return powerVec;
    }
    @Override
    public void runOpMode() {

        // Initialize PCDrivePowers x/y
        File rFile = AppUtil.getInstance().getSettingsFile("resultFile.txt");
        double result = Double.parseDouble(ReadWriteFile.readFile(rFile).trim());
        if (result >= 4) {
            playerX = 48;
            playerY = -96;
        } else {
            playerX = 48;
            playerY = 96;
        }

        // Initialize RoadRunner
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(180));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initialPose);
        try {
            Pose2d pose = mecanumDrive.readPoseFromDisk("xFile.txt", "yFile.txt", "hFile.txt");
            mecanumDrive.localizer.setPose(pose);
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }

        // Create stopwatch
        ElapsedTime runtime = new ElapsedTime();

        // Initialize hardware
        DcMotor intake = hardwareMap.get(DcMotorEx.class, "intake");

        Launch launch = new Launch(hardwareMap, "launch");
        Flip flip = new Flip(hardwareMap, "flip");
        Spindexer spindexer = new Spindexer(hardwareMap, "spindexer");

        // Create buttons

        Button flipButton = button(() -> gamepad2.y);

        button(() -> gamepad1.y).whenBecomesTrue(() -> {
            mecanumDrive.localizer.setPose(new Pose2d(0.01, 0.01, Math.toRadians(180)));
            try {
                mecanumDrive.writePoseToDisk("xFile.txt", "yFile.txt", "hFile.txt");
            } catch (FileNotFoundException e) {
                throw new RuntimeException(e);
            }
        });

        button(() -> gamepad1.b).whenBecomesTrue(() -> trajectoryActions = new ArrayList<>());

        button(() -> gamepad2.a).whenBecomesTrue(() -> runningActions.add(spindexer.spindex()));

        button(() -> gamepad1.dpad_up).whenBecomesTrue(() -> {
            closeOrFar = "close";
            runningActions.add(launch.launchAtSpeed(1900));
        });

        button(() -> gamepad1.dpad_down).whenBecomesTrue(() -> {
            closeOrFar = "far";
            runningActions.add(launch.launchAtSpeed(2100));
        });

        button(() -> gamepad2.b).whenBecomesTrue(() -> runningActions.add(launch.stopLauncher()));

        button(() -> gamepad1.back || gamepad2.back).whenBecomesTrue(() -> requestOpModeStop());

        button(() -> gamepad1.dpad_left).whenBecomesTrue(() -> {
            switch(closeOrFar) {
                case "close":
                    trajectoryActions.add(mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(45))
                            .build());
                    break;
                case "far":
                    trajectoryActions.add(mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(44, 0), Math.toRadians(25))
                            .build());
                    break;
            }
        });

        button(() -> gamepad1.dpad_right).whenBecomesTrue(() -> {
            switch(closeOrFar) {
                case "close":
                    trajectoryActions.add(mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(-45))
                            .build());
                    break;
                case "far":
                    trajectoryActions.add(mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(44, 0), Math.toRadians(-35))
                            .build());
                    break;
            }
        });

        button(() -> gamepad1.left_bumper).whenBecomesTrue(() -> {
            playerX = 48;
            playerY = -96;
        });

        button(() -> gamepad1.right_bumper).whenBecomesTrue(() -> {
            playerX = 48;
            playerY = 96;
        });

        Button x2 = button(() -> gamepad2.x);

        Button x2Toggle = x2.toggleOnBecomesTrue();
        x2Toggle.whenBecomesTrue(() -> spindexer.engageTrim());
        x2Toggle.whenBecomesFalse(() -> spindexer.disengageTrim());
        x2Toggle.whenTrue(() -> spindexer.trimSpindexer(gamepad2.left_trigger, gamepad2.right_trigger));
        x2Toggle.whenFalse(() -> intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger));

        flipButton.whenBecomesFalse(() -> runningActions.add(flip.flipDown()));

        flipButton.whenBecomesTrue(() -> runningActions.add(flip.flipUp()));

        // Init limbo
        waitForStart();

        // Reset stopwatch
        runtime.reset();

        // Main loop
        while (!isStopRequested()) {

            // Create new dashboard packet
            TelemetryPacket packet = new TelemetryPacket();


            // Update pose
            mecanumDrive.updatePoseEstimate();

            // Update running actions
            Iterator<Action> actionIterator = runningActions.iterator();
            while (actionIterator.hasNext()) {
                Action action = actionIterator.next();
                action.preview(packet.fieldOverlay());

                if (!action.run(packet)) {
                    actionIterator.remove();
                }
            }


            // Update gamepads
            BindingManager.update();


            Iterator<Action> trajectoryIterator = trajectoryActions.iterator();
            while (trajectoryIterator.hasNext()) {
                Action action = trajectoryIterator.next();
                action.preview(packet.fieldOverlay());

                if (!action.run(packet)) {
                    trajectoryIterator.remove();
                }
            }

            // Switch drive mode
            if (trajectoryActions.isEmpty()) {
                switch (driveID) {

                    case 1:

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

                    case 2:

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

                    case 3:

                        // Player centric
                        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                                PCDrivePowers(
                                        mecanumDrive.localizer.getPose(),
                                        -gamepad1.left_stick_x,
                                        -gamepad1.left_stick_y
                                ),
                                -gamepad1.right_stick_x
                        ));
                        packet.put("DriveSystem is", "Player Centric");
                        break;

                    default:
                        throw new DriveException("I have no clue how you did this");
                }
            }

            // Add data to telemetry
            deltaTime = runtime.milliseconds() - prevTime;
            prevTime = runtime.milliseconds();
            telemetry.addData("Tick time (ms)", deltaTime);
            packet.put("Tick time (ms)", deltaTime);
            telemetry.addData("Shooter velocity: ", launch.getLaunchSpeed());
            packet.put("Shooter velocity", launch.getLaunchSpeed());
            telemetry.addData("Shooter current: ", launch.getLaunchCurrent(CurrentUnit.MILLIAMPS));
            packet.put("Shooter current: ", launch.getLaunchCurrent(CurrentUnit.MILLIAMPS));
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), mecanumDrive.localizer.getPose());

            // Send telemetry
            dash.sendTelemetryPacket(packet);
            telemetry.update();
        }

        BindingManager.reset();

        try {
            mecanumDrive.writePoseToDisk("xFile.txt", "yFile.txt", "hFile.txt");
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
    }
}
