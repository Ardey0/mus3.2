package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Tilt {
    private ServoImplEx servo = null;

    // TODO: calibreaza
    public final double UP = 0,
                        DOWN = 1,
                        STRAIGHT = 0.5;

    public Tilt(@NonNull HardwareMap hardwareMap) {
        this.servo = hardwareMap.get(ServoImplEx.class, "tilt");
        // look into: servo.setPwmRange()
    }

    public void setPosition(double position) {
        servo.setPosition(position);
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
