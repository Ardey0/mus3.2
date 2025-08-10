package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class Lift {
    private DcMotorEx leftMotor = null;
    private DcMotorEx rightMotor = null;
    private DcMotorEx encoder = null;
    private PIDFController controller = null;

    private static double kP = 0, kI = 0, kD = 0, kF = 0;
    private final int tolerance = 0;
    private int target = 0;
    // TODO: calibreaza
    public final int    DOWN = 0,
                        MAX_HEIGHT = 3400,
                        HIGH_BASKET = 3200,
                        LOW_BASKET = 1700,
                        PICKUP_HIGH = 750,
                        PICKUP_LOW = 500,
                        PICKUP_FENCE = 220,
                        HIGH_RUNG = 1100,
                        LOW_RUNG = 300,
                        ASCENT = 2750;

    public Lift(@NonNull HardwareMap hardwareMap) {
        this.leftMotor = hardwareMap.get(DcMotorEx.class, "leftLift");
        this.rightMotor = hardwareMap.get(DcMotorEx.class, "rightLift");
        this.encoder = hardwareMap.get(DcMotorEx.class, "liftEncoder"); // de schimbat in motorul cu encoderul

        this.controller = new PIDFController(kP, kI, kD, kF);
        this.controller.setTolerance(tolerance);

        this.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setTarget(int target) {
        this.target = target;
    }

    public void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void runToTarget() {
        double position = -encoder.getCurrentPosition();
        double power = controller.calculate(position, target);

        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public int getPosition() {
        return encoder.getCurrentPosition();
    }

    public void resetEncoder() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updateCoefficients(double kP, double kI, double kD, double kF) {
        // doar in debugging
        controller.setPIDF(kP, kI, kD, kF);
    }
}
