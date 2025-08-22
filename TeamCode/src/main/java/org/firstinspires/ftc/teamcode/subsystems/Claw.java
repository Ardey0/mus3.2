package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Claw {
    private ServoImplEx servo = null;
    private AnalogInput encoder = null;
    public static final double  OPEN = 0,
                                CLOSED = 1;


    public enum State {
        OPEN,
        CLOSED_WITH_SAMPLE,
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

    public double getPosition() {
        return encoder.getVoltage() / 3.3 * 360;
    }

    public State getState() {
        // TODO: calibreaza
        State state;
        double angle = getPosition();
        if (angle < 190) {
            state = State.CLOSED_NO_SAMPLE;
        } else if (angle >= 190 && angle < 225) {
            state = State.OPEN;
        } else {
            state = State.CLOSED_WITH_SAMPLE;
        }
        return state;
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
