package org.firstinspires.ftc.teamcode.TeamOpModes;



import static dev.nextftc.bindings.Bindings.button;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.jetbrains.annotations.Contract;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Arrays;
import org.firstinspires.ftc.teamcode.TeamOpModes.ActionConfig.*;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;

@Autonomous
public class TestAuto extends LinearOpMode {

    public static <T> ArrayList<T> addAndMoveRight(ArrayList<T> inputArray, int index, T thingToAdd) {
        ArrayList<T> result = new ArrayList<>(inputArray.size() + 1);
        result.addAll(inputArray.subList(0, index));
        result.add(thingToAdd);
        result.addAll(inputArray.subList(index, inputArray.size()));
        return result;
    }

    // Helping myself out by returning a bit more data from findPosition()
    static class ReturnPair {
        private int valueA;
        private Pose2d valueB;

        // Public constructors
        public ReturnPair(int valueA, Pose2d valueB) {
            this.valueA = valueA;
            this.valueB = valueB;
        }
        public ReturnPair() {
            this.valueA = 0;
            this.valueB = new Pose2d(0, 0, 0);
        }

        // Public helpers
        public int getValueA() {
            return this.valueA;
        }
        public void setValueA(int valueA) {
            this.valueA = valueA;
        }
        public Pose2d getValueB() {
            return this.valueB;
        }
        public void setValueB(Pose2d valueB) {
            this.valueB = valueB;
        }
    }

    // I don't understand why this isn't in the ArrayList class by default. It's inefficient, but it works.
    public static <T> ArrayList<T> reverseList(ArrayList<T> input) {
        ArrayList<T> result = new ArrayList<>(input.size());
        for (int i = input.size() - 1; i >= 0; i--) {
            result.add(input.get(i));
        }
        return result;
    }

    // Helper to find our starting position based on binary inputs
    @NonNull
    @Contract("_ -> new")
    private ReturnPair findPosition(@NonNull ArrayList<Boolean> inputsTemp) {
        ArrayList<Boolean> inputs = reverseList(inputsTemp);
        // Convert an array of booleans to an integer. I should really make this a helper.
        int result = 0;
        for (int i = 0; i < inputs.size() && i < 32; i++) {
            if (inputs.get(i)) {
                result |= (1 << i);
            }
        }

        // Declare a new pose so I don't have to do it every time.
        Pose2d startPose;

        // Actually find the damn pose
        switch (result) {
            case 0:
                startPose = new Pose2d(64, -12, Math.toRadians(180));
                break;
            case 1:
                startPose = new Pose2d(64, -28, Math.toRadians(180));
                break;
            case 2:
                startPose = new Pose2d(-44, -54, Math.toRadians(90));
                break;
            case 3:
                startPose = new Pose2d(-64, -36, Math.toRadians(0));
                break;
            case 4:
                startPose = new Pose2d(60, 12, Math.toRadians(-40));
                break;
            case 5:
                startPose = new Pose2d(-36, 60, Math.toRadians(-90));
                break;
            case 6:
                startPose = new Pose2d(-60, 46, Math.toRadians(-50));
                break;
            default:
                throw new IllegalArgumentException("You asshats put in an invalid number. THERE IS LITERALLY ONLY ONE INCORRECT NUMBER.");
        }
        return new ReturnPair(result, startPose);
    }

    // Declare and initialize variables to prepare for running
    ArrayList<Boolean> poseMap = new ArrayList<>();
    int i = 0;
    int delay = 0;
    private FtcDashboard dash = FtcDashboard.getInstance();

    void setTrueAndIncrement() {
        poseMap.add(i, true);
        i++;
    }

    void setFalseAndIncrement() {
        poseMap.add(i, false);
        i++;
    }


    public void runOpMode() {

        // Initialize hardware
        Flip flip = new Flip(hardwareMap, "flip");
        Launch launch = new Launch(hardwareMap, "launch");
        Spindexer spindexer = new Spindexer(hardwareMap, "spindexer");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");


        // Initialize buttons
        Button increment = button(() -> gamepad1.dpad_up);
        Button decrement = button(() -> gamepad1.dpad_down);

        increment.inLayer("initPose")
                .whenBecomesTrue(() -> setTrueAndIncrement())
                .inLayer("initDelay")
                .whenBecomesTrue(() -> delay++);

        decrement.inLayer("initPose")
                .whenBecomesTrue(() -> setFalseAndIncrement())
                .inLayer("initDelay")
                .whenBecomesTrue(() -> delay--);

        // Get an array of 3 binary numbers that signify where we start
        BindingManager.setLayer("initPose");
        while (poseMap.size() < 3 && !isStopRequested()) {
            BindingManager.update();
            for (int j = 0; j < poseMap.size(); j++) {
                telemetry.addData("bit " + j + ": ", poseMap.get(j));
            }
            telemetry.update();
        }

        // Configure delay for auto
        BindingManager.setLayer("initDelay");

        while (!gamepad1.b && !isStopRequested() && !opModeIsActive()) {
            BindingManager.update();
            telemetry.addData("Delay: ", delay);
            telemetry.update();
        }

        // Initialize RoadRunner classes
        ReturnPair startValues = findPosition(poseMap);
        Pose2d initPose = startValues.getValueB();
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initPose);

        Action traj1, traj2;
        Pose2d pose1, pose2;
        int launchSpeed = 2100;
        switch (startValues.getValueA()) {
            case 0:
            case 1:
                pose1 = new Pose2d(new Vector2d(54, -12), Math.toRadians(20));
                traj1 = mecanumDrive.actionBuilder(initPose).strafeToLinearHeading(pose1.position, pose1.heading).build();
                break;
            case 2:
            case 3:
                pose1 = new Pose2d(new Vector2d(-16, -16), Math.toRadians(40));
                launchSpeed = 1800;
                traj1 = mecanumDrive.actionBuilder(initPose).strafeToLinearHeading(pose1.position, pose1.heading).build();
                break;
            case 4:
                pose1 = new Pose2d(new Vector2d(60, 10), Math.toRadians(-30));
                traj1 = mecanumDrive.actionBuilder(initPose).turnTo(Math.toRadians(-30)).build();
                break;
            case 5:
            case 6:
                pose1 = new Pose2d(new Vector2d(-16, 16), Math.toRadians(-45));
                traj1 = mecanumDrive.actionBuilder(initPose).strafeToLinearHeading(pose1.position, pose1.heading).build();
                launchSpeed = 1800;
                break;
            default:
                throw new RuntimeException("How did you get this far without throwing another exception?");
        }

        if (startValues.getValueA() >= 4) {
            pose2 = new Pose2d(-60, 36, Math.toRadians(0));
        } else {
            pose2 = new Pose2d(-60, -36, Math.toRadians(0));
        }

        traj2 = mecanumDrive.actionBuilder(pose1).strafeToSplineHeading(pose2.position, pose2.heading).build();

        TelemetryPacket packet = new TelemetryPacket();


        // Create the main action
        Action mainAction =
                new SequentialAction(
                        new SleepAction(delay),
                        new ParallelAction(
                                launch.setLaunchSpeed(launchSpeed),
                                launch.launchUsingStoredSpeed(),
                                traj1
                        ),
                        flip.flipUp(),
                        new SleepAction(0.75),
                        flip.flipDown(),
                        new SleepAction(0.75),
                        spindexer.spindex(),
                        new SleepAction(0.75),
                        flip.flipUp(),
                        new SleepAction(0.75),
                        flip.flipDown(),
                        new SleepAction(0.75),
                        spindexer.spindex(),
                        new SleepAction(0.75),
                        flip.flipUp(),
                        new SleepAction(0.75),
                        flip.flipDown(),
                        new ParallelAction(
                                launch.stopLauncher(),
                                traj2
                        )
                );


        packet.fieldOverlay().setStroke("#3F51B5");
        mainAction.preview(packet.fieldOverlay());
        dash.sendTelemetryPacket(packet);

        // Init limbo
        waitForStart();

        // Start spintake
        intake.setPower(1);

        // Main loop
        while (!isStopRequested()) {
            packet = new TelemetryPacket();
            if (!mainAction.run(packet)) {
                break;
            }
            dash.sendTelemetryPacket(packet);
        }

        sleep(2000);
        mecanumDrive.updatePoseEstimate();
        try {
            mecanumDrive.writePoseToDisk("xFile.txt", "yFile.txt", "hFile.txt");
            File rFile = AppUtil.getInstance().getSettingsFile("resultFile.txt");

            ReadWriteFile.writeFile(rFile, String.valueOf(startValues.getValueA()));
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
    }
}
