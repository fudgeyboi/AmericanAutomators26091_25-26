package org.firstinspires.ftc.teamcode.TeamOpModes;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class ActionConfig {
    public static class Flip {
        private final double retractedValue = 0.1, extendedValue = 0.52;
        private final Servo flipServo;
        public Flip(HardwareMap hardwareMap, String servoName) {
            flipServo = hardwareMap.get(Servo.class, servoName);
            flipServo.setDirection(Servo.Direction.REVERSE);
            flipServo.setPosition(retractedValue);
        }
        public Action flipUp() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    flipServo.setPosition(extendedValue);
                    return false;
                }
            };
        }
        public Action flipDown() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    flipServo.setPosition(retractedValue);
                    return false;
                }
            };
        }
    }
    public static class Launch {
        private int launchSpeed = 0;
        private final DcMotorEx launchMotor;
        public Launch(HardwareMap hardwareMap, String motorName) {
            launchMotor = hardwareMap.get(DcMotorEx.class, motorName);
            launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launchMotor.setCurrentAlert(8, CurrentUnit.AMPS);
            launchMotor.setVelocityPIDFCoefficients(32, 2.5, 0.5, 1.5);
            launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public Action setLaunchSpeed(int launchSpeed) {
            this.launchSpeed = launchSpeed;
            return packet -> false;
        }
        public Action launchAtSpeed(int launchSpeed) {
            return new Action() {
                private boolean initialized = false;
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if (!initialized) {
                        launchMotor.setVelocity(launchSpeed);
                        initialized = true;
                    }

                    double vel = launchMotor.getVelocity();
                    telemetryPacket.put("Shooter velocity is", vel);
                    return vel < (launchSpeed * 0.95);
                }
            };
        }
        public Action launchUsingStoredSpeed() {
            return new Action() {
                private boolean initialized = false;
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if (!initialized) {
                        launchMotor.setVelocity(Launch.this.launchSpeed);
                        initialized = true;
                    }

                    double vel = launchMotor.getVelocity();
                    telemetryPacket.put("Shooter velocity is", vel);
                    return vel < (Launch.this.launchSpeed * 0.95);
                }
            };
        }
    }
}
