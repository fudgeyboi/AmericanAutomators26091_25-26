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

import java.lang.reflect.GenericArrayType;
import java.lang.reflect.GenericDeclaration;
import java.util.ArrayList;
import java.util.PrimitiveIterator;

@Autonomous
public class UnifiedAuto extends LinearOpMode {
    public static <T> ArrayList<T> addAndMoveRight(ArrayList<T> inputArray, int index, T thingToAdd) {
        ArrayList<T> leftArray = (ArrayList<T>) inputArray.subList(0, index);
        ArrayList<T> rightArray = (ArrayList<T>) inputArray.subList(index, inputArray.size());
        leftArray.add(thingToAdd);
        leftArray.addAll(rightArray);
        return leftArray;
    }
    private static class ReturnPair {
        private int valueA;
        private Pose2d valueB;
        public int getValueA() {
            return valueA;
        }
        ReturnPair(int valueA, Pose2d valueB) {
            this.valueA = valueA;
            this.valueB = valueB;

        }
        public void setValueA(int valueA) {
            this.valueA = valueA;
        }
        public void setValueB(Pose2d valueB) {
            this.valueB = valueB;
        }
        public Pose2d getValueB() {
            return valueB;
        }
    }
    @NonNull
    @Contract("_ -> new")
    private ReturnPair findPosition(@NonNull boolean[] inputsTemp) {
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
        Pose2d startPose = new Pose2d(0, 0, 0);
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
        }
        return new ReturnPair(result, startPose);
    }

    boolean[] poseMap = new boolean[3];
    boolean dpadDownPrev;
    boolean dpadUpPrev;
    public void runOpMode() {
        Servo flip = hardwareMap.get(Servo.class, "flip");
        flip.setDirection(Servo.Direction.REVERSE);
        DcMotorEx launch = hardwareMap.get(DcMotorEx.class, "launch");
        launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launch.setCurrentAlert(8, CurrentUnit.AMPS);
        launch.setVelocityPIDFCoefficients(24, 0.75, 1, 2.5);
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
        Pose2d initPose = findPosition(poseMap).getValueB();
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initPose);
        ArrayList<TrajectoryActionBuilder> trajectoryArray = new ArrayList<TrajectoryActionBuilder>();
        trajectoryArray.add(mecanumDrive.actionBuilder(initPose).strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(-40)));

        ArrayList<Action> actionsList = new ArrayList<Action>();

        for (Action item : actionsList) {
            Actions.runBlocking(item);
        }
        waitForStart();
    }
}
