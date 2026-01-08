package org.firstinspires.ftc.teamcode.TeamOpModes;


import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.jetbrains.annotations.Contract;
import java.util.ArrayList;
import java.util.Arrays;
import org.firstinspires.ftc.teamcode.TeamOpModes.ActionConfig.*;

@Autonomous
public class UnifiedAuto extends LinearOpMode {

    // I don't know why it doesn't have this already, but we do now!
    public static <T> ArrayList<T> addAndMoveRight(ArrayList<T> inputArray, int index, T thingToAdd) {
        ArrayList<T> result = new ArrayList<>(inputArray.size() + 1);
        result.addAll(inputArray.subList(0, index));
        result.add(thingToAdd);
        result.addAll(inputArray.subList(index, inputArray.size()));
        return result;
    }

    // Helping myself out by returning a bit more data from findPosition()
    static class ReturnPair {

        // Public constructor
        public ReturnPair(int valueA, Pose2d valueB) {
            this.valueA = valueA;
            this.valueB = valueB;
        }
        public ReturnPair() {
            this.valueA = 0;
            this.valueB = new Pose2d(0, 0, 0);
        }

        private int valueA;
        private Pose2d valueB;

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
                startPose = new Pose2d(64, -14, Math.toRadians(180));
                break;
            case 1:
                startPose = new Pose2d(64, -32, Math.toRadians(180));
                break;
            case 2:
                startPose = new Pose2d(-36, -48, Math.toRadians(90));
                break;
            case 3:
                startPose = new Pose2d(-64, -32, Math.toRadians(0));
                break;
            case 4:
                startPose = new Pose2d(60, 12, Math.toRadians(-40));
                break;
            case 5:
                startPose = new Pose2d(-40, 56, Math.toRadians(-90));
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


    public void runOpMode() throws InterruptedException {
        // Initialize variables
        int i = 0;

        // Initialize hardware
        Flip flip = new Flip(hardwareMap, "flip");
        Launch launch = new Launch(hardwareMap, "launch");

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

        Action traj1;
        // Add trajectories to the ArrayList
        if (startValues.getValueA() >= 4) {
            traj1 = mecanumDrive.actionBuilder(initPose)
                    .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(-40))
                    .build();
        } else {
            traj1 = mecanumDrive.actionBuilder(initPose)
                    .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(40))
                    .build();
        }

        // Init limbo
        waitForStart();

        // Run each action sequentially
        Actions.runBlocking(new SequentialAction(
                launch.launchAtSpeed(2240),
                traj1,
                flip.flipUp(),
                new SleepAction(1)
        ));
    }
}
