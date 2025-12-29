package org.firstinspires.ftc.teamcode.TeamOpModes;


// Importing OpMode class
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


// Import hardware classes
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

// Import computing libraries
import java.util.ArrayList;
import java.util.List;

// Import RoadRunner classes + dependencies
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drawing;

// Import custom classes
import org.firstinspires.ftc.teamcode.DriveException;



@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    double prevTime = 0;
    double deltaTime = 0;
    int driveID = 0;
    Pose2d backupPose;
    Vector2d PCDrivePowers(Pose2d pose, double gamepadx, double gamepady) {

        double heading = pose.heading.toDouble();
        double x = pose.position.x;
        double y = pose.position.y;

        double theta = Math.atan2(x + 48, y - 96);

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
    public void runOpMode() throws InterruptedException {

        // Initialize RoadRunner
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(-90));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initialPose);

        // Create stopwatch
        ElapsedTime runtime = new ElapsedTime();

        // Initialize hardware
        DcMotorEx launch = hardwareMap.get(DcMotorEx.class, "launch");
        launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launch.setCurrentAlert(8, CurrentUnit.AMPS);
        launch.setVelocityPIDFCoefficients(16, 1.5, 1, 2.5);
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

            // Create new dashboard packet
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
            if (gamepad1.y) {
                mecanumDrive.localizer.setPose(new Pose2d(0, 0, Math.toRadians(-90)));
            }

            // Queue actions
            {
                // Blue recenter && wind-up
                if (gamepad1.x && runningActions.isEmpty()) {
                    TrajectoryActionBuilder leftClose = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(135));
                    Action aimLeftClose = leftClose.build();
                    runningActions.add(aimLeftClose);
                    launch.setVelocity(2350);
                }

                // Red recenter && wind-up
                if (gamepad1.b && runningActions.isEmpty()) {
                    TrajectoryActionBuilder rightClose = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(45));
                    Action aimRightClose = rightClose.build();
                    runningActions.add(aimRightClose);
                    launch.setVelocity(2350);
                }
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
                                                    - (gamepad1.left_stick_x * java.lang.Math.cos(heading))),
                                            ((gamepad1.left_stick_y * java.lang.Math.cos(heading))
                                                    + (gamepad1.left_stick_x * java.lang.Math.sin(heading)))
                                    ),
                                    -gamepad1.right_stick_x
                            ));
                            packet.put("DriveSystem is", "Field Centric");
                            break;

                        case 2:
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
            if (gamepad1.dpad_down) {
                driveID = 0;
            }
            if (gamepad1.dpad_up) {
                driveID = 1;
            }
            if (gamepad1.dpad_left) {
                driveID = 2;
            }

            // Misc motors block
            {
                if (gamepad2.b) {
                    launch.setVelocity(0);
                }
            }

            // Servo block
            {
                if (gamepad2.a) {
                    flip.setPosition(0.52);
                } else {
                    flip.setPosition(0.1);
                }
                spindexer.setPower((gamepad2.right_stick_x / 5) + (gamepad2.left_stick_x / 5));
            }

            intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

            // Add data to telemetry
            deltaTime = runtime.milliseconds() - prevTime;
            prevTime = runtime.milliseconds();
            packet.put("Tick time (ms)", deltaTime);
            packet.put("Shooter velocity (ticks/sec)", launch.getVelocity());
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), mecanumDrive.localizer.getPose());
            telemetry.addData("Velocity (ticks/sec)", launch.getVelocity());

            // Send telemetry
            dash.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
