package org.firstinspires.ftc.teamcode.TeamOpModes;


// Importing OpMode class
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Import hardware classes
import com.qualcomm.robotcore.util.ElapsedTime;

// Import computing libraries

// Import RoadRunner classes + dependencies
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


// Import custom classes
import org.firstinspires.ftc.teamcode.TankDrive;


// Import NextFTC classes
import dev.nextftc.bindings.BindingManager;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    // Declare and initialize global variables
    private FtcDashboard dash = FtcDashboard.getInstance();
    @Override
    public void runOpMode() {

        TankDrive tankDrive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Create stopwatch
        ElapsedTime runtime = new ElapsedTime();

        // Init limbo
        waitForStart();

        // Reset stopwatch
        runtime.reset();

        // Main loop
        while (!isStopRequested()) {

            // Create new dashboard packet
            TelemetryPacket packet = new TelemetryPacket();

            tankDrive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            0
                    ),
                    -gamepad1.right_stick_x
            ));

            // Update gamepads
            BindingManager.update();

            // Add data to telemetry
            double deltaTime = runtime.milliseconds();
            runtime.reset();
            telemetry.addData("Tick time (ms)", deltaTime);
            packet.put("Tick time (ms)", deltaTime);

            // Send telemetry
            dash.sendTelemetryPacket(packet);
            telemetry.update();

        }

        BindingManager.reset();

    }
}
