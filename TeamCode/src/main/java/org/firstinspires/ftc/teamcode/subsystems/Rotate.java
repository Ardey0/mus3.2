package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Rotate {
    private ServoImplEx servo = null;

    // TODO: calibreaza
    public final double STRAIGHT = 0.5,
                        LEFT = 0,
                        RIGHT = 1;

    public Rotate(@NonNull HardwareMap hardwareMap) {
        this.servo = hardwareMap.get(ServoImplEx.class, "rotate");
        // look into: servo.setPwmRange()
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public void trackTarget(Limelight limelight) {
        servo.setPosition(limelight.getAngle() / 180);
    }

    public double getPosition() {
        return servo.getPosition(); // fara encoder
    }

    public void enable() {
        if (!servo.isPwmEnabled()) {
            servo.setPwmEnable();
        }
    }

    public void disable() {
        if (servo.isPwmEnabled()) {
            servo.setPwmDisable();
        }
    }
}
