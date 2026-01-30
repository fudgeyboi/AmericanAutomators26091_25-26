package org.firstinspires.ftc.teamcode.TeamOpModes;


// Importing OpMode class
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Import hardware classes
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drawing;

// Import custom classes
import org.firstinspires.ftc.teamcode.DriveException;
import org.firstinspires.ftc.teamcode.TeamOpModes.ActionConfig.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    // Declare and initialize global variables
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private List<Action> trajectoryActions = new ArrayList<>();
    double prevTime = 0;
    double deltaTime = 0;
    int driveID = 3;
    boolean a2WasPressed = false;
    String closeOrFar = "close";
    boolean yWasPressed = false;
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


        // Init limbo
        waitForStart();

        // Reset stopwatch
        runtime.reset();

        // Main loop
        while (opModeIsActive() && !(gamepad1.back || gamepad2.back)) {

            // Create new dashboard packet
            TelemetryPacket packet = new TelemetryPacket();

            // Update running actions
            Iterator<Action> iterator = runningActions.iterator();
            while (iterator.hasNext()) {
                Action action = iterator.next();
                action.preview(packet.fieldOverlay());

                if (!action.run(packet)) {
                    iterator.remove();
                }
            }


            iterator = trajectoryActions.iterator();
            while (iterator.hasNext()) {
                Action action = iterator.next();
                action.preview(packet.fieldOverlay());

                if (!action.run(packet)) {
                    iterator.remove();
                }
            }


            // Update pose + reset if necessary
            mecanumDrive.updatePoseEstimate();
            if (gamepad1.y && !yWasPressed) {
                mecanumDrive.localizer.setPose(new Pose2d(0.01, 0.01, Math.toRadians(180)));
                try {
                    mecanumDrive.writePoseToDisk("xFile.txt", "yFile.txt", "hFile.txt");
                } catch (FileNotFoundException e) {
                    throw new RuntimeException(e);
                }
            }

            yWasPressed = gamepad1.y;

            // Queue actions
            if (gamepad1.dpad_up) {
                closeOrFar = "close";
            }
            if (gamepad1.dpad_down) {
                closeOrFar = "far";
            }
            if (gamepad1.dpad_left) {
                switch(closeOrFar) {
                    case "close":
                        runningActions.add(launch.launchAtSpeed(1900));
                        trajectoryActions.add(mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                                .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(45))
                                .build());
                        break;
                    case "far":
                        runningActions.add(launch.launchAtSpeed(2100));
                        trajectoryActions.add(mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                                .strafeToLinearHeading(new Vector2d(44, 0), Math.toRadians(25))
                                .build());
                        break;
                }
            }
            if (gamepad1.dpad_right) {
                switch(closeOrFar) {
                    case "close":
                        runningActions.add(launch.launchAtSpeed(1900));
                        trajectoryActions.add(mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                                .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(-45))
                                .build());
                        break;
                    case "far":
                        runningActions.add(launch.launchAtSpeed(2100));
                        trajectoryActions.add(mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                                .strafeToLinearHeading(new Vector2d(44, 0), Math.toRadians(-35))
                                .build());
                        break;
                }
            }
            if (gamepad1.b) {
                trajectoryActions = new ArrayList<>();
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

            // Control drive system switching
            // (not implemented)

            // Misc motors block
            {
                if (gamepad2.b) {
                    runningActions.add(launch.stopLauncher());
                }
                if (gamepad2.a && !a2WasPressed) {
                    runningActions.add(spindexer.spindex());
                }
                a2WasPressed = gamepad2.a;
                intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
            }

            // Servo block
            {
                if (gamepad2.y) {
                    runningActions.add(flip.flipUp());
                } else {
                    runningActions.add(flip.flipDown());
                }
            }

            // Add data to telemetry
            deltaTime = runtime.milliseconds() - prevTime;
            prevTime = runtime.milliseconds();
            packet.put("Tick time (ms)", deltaTime);
            telemetry.addData("Tick time (ms)", deltaTime);
            packet.put("Shooter velocity", launch.getLaunchSpeed());
            telemetry.addData("Shooter velocity: ", launch.getLaunchSpeed());
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), mecanumDrive.localizer.getPose());

            // Send telemetry
            dash.sendTelemetryPacket(packet);
            telemetry.update();
        }
        try {
            mecanumDrive.writePoseToDisk("xFile.txt", "yFile.txt", "hFile.txt");
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
    }
}
