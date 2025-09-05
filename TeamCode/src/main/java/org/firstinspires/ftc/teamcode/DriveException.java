package org.firstinspires.ftc.teamcode;

public class DriveException extends RuntimeException {
    public DriveException(String message, Throwable err) {
        super(message, err);
    }
}
