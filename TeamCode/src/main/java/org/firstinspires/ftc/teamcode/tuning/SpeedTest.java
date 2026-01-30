package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.TeamOpModes.ActionConfig.*;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

@TeleOp
public class SpeedTest extends LinearOpMode {

    private List<Action> runningActions = new ArrayList<>();
    int i = 0;

    @Override
    public void runOpMode() {

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "launch");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocityPIDFCoefficients(48, 0.2, 1, 12);
        Flip flip = new Flip(hardwareMap, "flip");

        waitForStart();

        while (!isStopRequested()) {

            if (gamepad1.dpad_up && i < 2200) {
                i += 20;
                motor.setVelocity(i);
            } else if (gamepad1.dpad_down && i >= 20) {
                i -= 20;
                motor.setVelocity(i);
            }
            if (gamepad1.right_bumper) {
                runningActions.add(flip.flipUp());
            }
            if (gamepad1.left_bumper) {
                runningActions.add(flip.flipDown());
            }

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


            telemetry.addData("speed:", i);
            telemetry.update();
            sleep(100);
        }
    }
}
