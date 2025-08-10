package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Claw {
    private ServoImplEx servo = null;
    private AnalogInput encoder = null;
    public enum State {
        OPEN,
        CLOSED,
        CLOSED_NO_SAMPLE
    };

    public Claw(@NonNull HardwareMap hardwareMap) {
        this.servo = hardwareMap.get(ServoImplEx.class, "claw");
        this.encoder = hardwareMap.get(AnalogInput.class, "encoderClaw");
        // look into: servo.setPwmRange()
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public void setPosition(State position) {
        servo.setPosition(position == State.OPEN ? 0 : 1); // TODO: calibreaza
    }

    public double getPosition() {
        return encoder.getVoltage() / 3.3 * 360;
    }

    public State getState() {
        // TODO: calibreaza
        State state;
        double angle = getPosition();
        if (angle > 280 && angle < 300) {
            state = State.CLOSED;
        } else if (angle > 300) {
            state = State.CLOSED_NO_SAMPLE;
        } else {
            state = State.OPEN;
        }
        return state;
    }

    public void enable() {
        if (!servo.isPwmEnabled()) {
            servo.setPwmEnable();
        } // else throw new RuntimeException("Servo is already enabled!");
    }

    public void disable() {
        if (servo.isPwmEnabled()) {
            servo.setPwmDisable();
        } // else throw new RuntimeException("Servo is already enabled!");
    }
}
