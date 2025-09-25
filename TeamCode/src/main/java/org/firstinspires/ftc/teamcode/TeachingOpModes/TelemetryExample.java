package org.firstinspires.ftc.teamcode.TeachingOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TelemetryExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Dpad Down", gamepad1.dpad_down);
            telemetry.addData("Dpad Right", gamepad1.dpad_right);
            telemetry.addData("Dpad Up", gamepad1.dpad_up);
            telemetry.addData("Dpad Left", gamepad1.dpad_left);
            telemetry.update();
        }
    }
}
