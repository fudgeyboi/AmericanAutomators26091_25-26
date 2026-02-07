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
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.TeamOpModes.ActionConfig.*;


// Import NextFTC classes
import static dev.nextftc.bindings.Bindings.*;
import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class HammerHeadTeleOp extends LinearOpMode {

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
