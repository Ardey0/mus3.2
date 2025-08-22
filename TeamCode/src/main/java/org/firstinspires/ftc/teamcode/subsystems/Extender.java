package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDController;

public class Extender {
    private DcMotorEx motor = null;
    private PIDController controller = null;

    private static final double kP = 0.05, kI = 0, kD = 0.0015;
    private final int tolerance = 0;
    private int target = 0;
    // TODO: calibreaza
    public static final int RETRACTED = 0,
                            PICKUP = 200,
                            SCORE_RUNG = 350,
                            SCUIPA = 200,
                            MAX_LENGTH = 390;

    public Extender(@NonNull HardwareMap hardwareMap) {
        this.motor = hardwareMap.get(DcMotorEx.class, "extender");
        this.controller = new PIDController(kP, kI, kD);
        this.controller.setTolerance(tolerance);

        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTarget(int target) {
        this.target = target;
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void runToTarget() {
        double position = motor.getCurrentPosition();
        double power = controller.calculate(position, target);

        motor.setPower(power);
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public int delta() {
        return Math.abs(target - motor.getCurrentPosition());
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updateCoefficients(double kP, double kI, double kD) {
        // doar in debugging
        controller.setPID(kP, kI, kD);
    }
}
