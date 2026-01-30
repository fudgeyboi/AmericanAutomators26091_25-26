package org.firstinspires.ftc.teamcode.TeamOpModes;



import androidx.annotation.NonNull;

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

@Autonomous
public class UnifiedAuto extends LinearOpMode {

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

    ArrayList<Boolean> poseMap = new ArrayList<>(Arrays.asList(false, false, false));
    boolean dpadDownPrev = false;
    boolean dpadUpPrev = false;


    public void runOpMode() {
        // Initialize variables
        MecanumDrive.Params PARAMS = new MecanumDrive.Params();

        MecanumKinematics kinematics = new MecanumKinematics(
                PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

        MinVelConstraint constraint = new MinVelConstraint(Arrays.asList(kinematics.new WheelVelConstraint(5), new AngularVelConstraint(PARAMS.maxAngVel)));

        int i = 0;

        // Initialize hardware
        Flip flip = new Flip(hardwareMap, "flip");
        Launch launch = new Launch(hardwareMap, "launch");
        Spindexer spindexer = new Spindexer(hardwareMap, "spindexer");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        // Get an array of 3 binary numbers that signify where we start
        while (i < poseMap.size() && !isStopRequested()) {
            if (gamepad1.dpad_up && !dpadUpPrev) {
                poseMap.set(i, true);
                i++;
            }
            if (gamepad1.dpad_down && !dpadDownPrev) {
                poseMap.set(i, false);
                i++;
            }
            dpadUpPrev = gamepad1.dpad_up;
            dpadDownPrev = gamepad1.dpad_down;
            telemetry.addData("bit 1", poseMap.get(0));
            telemetry.addData("bit 2", poseMap.get(1));
            telemetry.addData("bit 3", poseMap.get(2));
            telemetry.update();
        }

        // Initialize RoadRunner classes
        ReturnPair startValues = findPosition(poseMap);
        Pose2d initPose = startValues.getValueB();
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initPose);

        Action traj1, traj2, traj3, traj4;
        Pose2d pose1, pose2, pose3;
        int launchSpeed = 2100;
        switch (startValues.getValueA()) {
            case 0:
            case 1:
                pose1 = new Pose2d(new Vector2d(44, 0), Math.toRadians(20));
                break;
            case 2:
            case 3:
                pose1 = new Pose2d(new Vector2d(0, 0), Math.toRadians(40));
                launchSpeed = 1900;
                break;
            case 4:
                pose1 = new Pose2d(new Vector2d(44, 0), Math.toRadians(-30));
                break;
            case 5:
            case 6:
                pose1 = new Pose2d(new Vector2d(0, 0), Math.toRadians(-45));
                launchSpeed = 1900;
                break;
            default:
                throw new RuntimeException("How did you get this far without throwing another exception?");
        }
        traj1 = mecanumDrive.actionBuilder(initPose).strafeToLinearHeading(pose1.position, pose1.heading).build();

        if (startValues.getValueA() >= 4) {
            pose2 = new Pose2d(12, 12, Math.toRadians(-90));
            pose3 = new Pose2d(32, -52, Math.toRadians(90));
        } else {
            pose2 = new Pose2d(32, -26, Math.toRadians(-90));
            pose3 = new Pose2d(32, -60, Math.toRadians(90));
        }

        traj2 = mecanumDrive.actionBuilder(pose1).strafeToSplineHeading(pose2.position, pose2.heading).build();
        traj3 = mecanumDrive.actionBuilder(pose2).strafeToConstantHeading(pose3.position, constraint).build();
        traj4 = mecanumDrive.actionBuilder(pose3).setTangent(Math.toRadians(90)).splineToSplineHeading(pose1, Math.toRadians(0)).build();

        // Init limbo
        waitForStart();

        // Start spintake
        intake.setPower(1);

        // Run each action sequentially
        Actions.runBlocking(
                new SequentialAction(
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
                        ),
                        new ParallelAction(
                                traj3,
                                new SequentialAction(
                                        new SleepAction(2),
                                        spindexer.spindex(9)
                                )
                        ),
                        new ParallelAction(
                                launch.launchUsingStoredSpeed(),
                                traj4
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
                        flip.flipDown()
                ));
        try {
            mecanumDrive.writePoseToDisk("xFile.txt", "yFile.txt", "hFile.txt");
            File rFile = AppUtil.getInstance().getSettingsFile("resultFile.txt");

            ReadWriteFile.writeFile(rFile, String.valueOf(startValues.getValueA()));
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
    }
}
