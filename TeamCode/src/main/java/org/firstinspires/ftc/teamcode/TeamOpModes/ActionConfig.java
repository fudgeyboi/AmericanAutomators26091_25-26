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
        private final double retractedValue = 0.1, extendedValue = 0.55;
        private final Servo flipServo;
        public Flip(HardwareMap hardwareMap, String servoName) {
            flipServo = hardwareMap.get(Servo.class, servoName);
            flipServo.setDirection(Servo.Direction.REVERSE);
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
            launchMotor.setCurrentAlert(5, CurrentUnit.AMPS);
            launchMotor.setVelocityPIDFCoefficients(48, 0.2, 1, 12);
            launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public double getLaunchSpeed() {
            return launchMotor.getVelocity();
        }
        public Action setLaunchSpeed(int launchSpeed) {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    Launch.this.launchSpeed = launchSpeed;
                    return false;
                }
            };
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
                    telemetryPacket.put("Shooter velocity", vel);
                    return vel < (launchSpeed * 0.975);
                }
            };
        }
        public double getLaunchCurrent(CurrentUnit currentUnit) {return launchMotor.getCurrent(currentUnit);}
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
                    telemetryPacket.put("Shooter velocity", vel);
                    return vel < (Launch.this.launchSpeed * 0.975);
                }
            };
        }
        public Action stopLauncher() {
            return new Action() {
                private boolean initialized = false;
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if (!initialized) {
                        launchMotor.setVelocity(0);
                        initialized = true;
                    }

                    double vel = launchMotor.getVelocity();
                    telemetryPacket.put("Shooter velocity", vel);
                    return vel > 100;
                }
            };
        }
    }

    public static class Spindexer {
        private int index = 0;
        private final DcMotorEx spindexerMotor;
        public Spindexer(HardwareMap hardwareMap, String motorName) {
            spindexerMotor = hardwareMap.get(DcMotorEx.class, motorName);
            spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexerMotor.setTargetPosition(0);
            spindexerMotor.setCurrentAlert(4, CurrentUnit.AMPS);
            spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spindexerMotor.setPower(1);
        }
        public void engageTrim() {
            spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            spindexerMotor.setPower(0);
        }
        public void disengageTrim() {
            spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            index = 0;
            spindexerMotor.setTargetPosition((int) Math.round(index * 1425.1 / 3));
            spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spindexerMotor.setPower(1);
        }
        public void trimSpindexer(double leftTrigger, double rightTrigger) {
            spindexerMotor.setPower((rightTrigger / 3   ) - (leftTrigger / 3));
        }
        public Action spindex() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    index++;
                    spindexerMotor.setTargetPosition((int) Math.round(index * 1425.1 / 3));
                    spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    return false;
                }
            };
        }
        public Action spindex(int timesToSpin) {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    index += timesToSpin;
                    spindexerMotor.setTargetPosition((int) Math.round(index * 1425.1 / 3));
                    spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    return false;
                }
            };
        }
    }
}
