package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.List;

@TeleOp(name = "LiftDebug", group = "Test")
@Config
public class LiftDebug extends LinearOpMode {
    private Lift lift = null;
    public static double kP = 0.045, kI = 0.01, kD = 0.00001, kF = 0.00008;
    public static int target = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime loopTime = new ElapsedTime();
        ElapsedTime runTime = new ElapsedTime();

        lift = new Lift(hardwareMap);

        telemetry.addData("Transmission Interval", telemetry.getMsTransmissionInterval());

        telemetry.setMsTransmissionInterval(10);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry.addLine("Initialized! Waiting for start...");
        telemetry.update();

        waitForStart();

        if (isStopRequested())
            return;
        else {
            gamepad1.rumble(250);
            gamepad2.rumble(250);
            runTime.reset();
        }

        while (opModeIsActive()) {
            loopTime.reset();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            if (gamepad2.a) {
                target = Lift.HIGH_BASKET;
            } else if (gamepad2.b) {
                target = Lift.HIGH_RUNG;
            } else if (gamepad2.x) {
                target = Lift.PICKUP_HIGH;
            } else if (gamepad2.y) {
                target = Lift.PICKUP_LOW;
            } else if (gamepad2.dpad_left) {
                target = 1900;
            } else if (gamepad2.dpad_down) {
                target = Lift.DOWN;
            } else if (gamepad2.dpad_up) {
                target = Lift.MAX_HEIGHT;
            } else if (gamepad2.dpad_right) {
                target = Lift.PICKUP_FENCE;
            } else if (gamepad2.right_bumper) {
                target = Lift.LOW_BASKET;
            } else if (gamepad2.left_bumper) {
                target = Lift.LOW_RUNG;
            } else if (gamepad2.start) {
                lift.resetEncoder();
            }

            lift.updateCoefficients(kP, kI, kD, kF);
            lift.setTarget(target);
            lift.runToTarget();

            telemetry.addData("Lift position:", lift.getPosition());
            telemetry.addData("Loop time:", loopTime.toString());
            telemetry.addData("Run time:", runTime.toString());
            telemetry.update();
        }
    }
}
