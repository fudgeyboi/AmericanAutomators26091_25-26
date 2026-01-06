package org.firstinspires.ftc.teamcode.TeamOpModes;


import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.jetbrains.annotations.Contract;
import java.util.ArrayList;

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
    private static class ReturnPair {

        // Public constructor
        public ReturnPair(int valueA, Pose2d valueB) {
            this.valueA = valueA;
            this.valueB = valueB;

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

    // Helper to find our starting position based on binary inputs
    @NonNull
    @Contract("_ -> new")
    private ReturnPair findPosition(@NonNull boolean[] inputsTemp) {

        // Convert an array of booleans to an integer. I should really make this a helper.
        int result = 0;
        boolean[] inputs = new boolean[inputsTemp.length];
        for (int i = 0; i < inputsTemp.length; i++) {
            inputs[i] = inputsTemp[inputsTemp.length - 1 - i];
        }
        for (int i = 0; i < inputs.length && i < 32; i++) {
            if (inputs[i]) {
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
    boolean[] poseMap = new boolean[3];
    boolean dpadDownPrev = false;
    boolean dpadUpPrev = false;


    public void runOpMode() {

        // Initialize hardware
        Servo flip = hardwareMap.get(Servo.class, "flip");
        flip.setDirection(Servo.Direction.REVERSE);
        DcMotorEx launch = hardwareMap.get(DcMotorEx.class, "launch");
        launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launch.setCurrentAlert(8, CurrentUnit.AMPS);
        launch.setVelocityPIDFCoefficients(24, 0.75, 1, 2.5);

        // Get an array of 3 binary numbers that signify where we start
        for (int i = 0; i < poseMap.length;) {
            if (gamepad1.dpad_up && !dpadUpPrev) {
                poseMap[i] = true;
                i++;
            }
            if (gamepad1.dpad_down && !dpadDownPrev) {
                poseMap[i] = false;
                i++;
            }
            dpadUpPrev = gamepad1.dpad_up;
            dpadDownPrev = gamepad1.dpad_down;
            telemetry.addData("bit 1", poseMap[0]);
            telemetry.addData("bit 2", poseMap[1]);
            telemetry.addData("bit 3", poseMap[2]);
            telemetry.update();
        }

        // Initialize RoadRunner classes
        ReturnPair startValues = findPosition(poseMap);
        Pose2d initPose = startValues.getValueB();
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initPose);

        // Make an ArrayList of TrajectoryActionBuilders so I don't have to build every one individually
        ArrayList<TrajectoryActionBuilder> trajectoryArray = new ArrayList<>();

        // Add trajectories to the ArrayList
        if (startValues.getValueA() >= 4) {
            trajectoryArray.add(mecanumDrive.actionBuilder(initPose).strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(-40)));
        } else {
            trajectoryArray.add(mecanumDrive.actionBuilder(initPose).strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(40)));
        }

        // Build each trajectory and add it to a new ArrayList
        ArrayList<Action> actionsList = new ArrayList<>();
        for (TrajectoryActionBuilder item : trajectoryArray) {
            actionsList.add(item.build());
        }

        // Init limbo
        waitForStart();

        // Run each action sequentially
        for (Action item : actionsList) {
            Actions.runBlocking(item);
        }
    }
}
