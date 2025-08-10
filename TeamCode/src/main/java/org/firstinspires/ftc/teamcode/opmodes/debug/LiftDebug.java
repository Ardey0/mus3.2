package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.List;

public class LiftDebug extends LinearOpMode {
    private Lift lift = null;
    public static double kP = 0, kI = 0, kD = 0, kF = 0;
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
                target = lift.HIGH_BASKET;
            } else if (gamepad2.b) {
                target = lift.HIGH_RUNG;
            } else if (gamepad2.x) {
                target = lift.PICKUP_HIGH;
            } else if (gamepad2.y) {
                target = lift.PICKUP_LOW;
            } else if (gamepad2.dpad_left) {
                target = lift.ASCENT;
            } else if (gamepad2.dpad_down) {
                target = lift.DOWN;
            } else if (gamepad2.dpad_up) {
                target = lift.MAX_HEIGHT;
            } else if (gamepad2.dpad_right) {
                target = lift.PICKUP_FENCE;
            } else if (gamepad2.right_bumper) {
                target = lift.LOW_BASKET;
            } else if (gamepad2.left_bumper) {
                target = lift.LOW_RUNG;
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
