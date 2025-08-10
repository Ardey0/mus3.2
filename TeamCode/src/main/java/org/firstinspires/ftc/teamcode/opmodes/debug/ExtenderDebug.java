package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Extender;

import java.util.List;

public class ExtenderDebug extends LinearOpMode {
    private Extender extender = null;
    public static double kP = 0, kI = 0, kD = 0, kF = 0;
    public static int target = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime loopTime = new ElapsedTime();
        ElapsedTime runTime = new ElapsedTime();

        extender = new Extender(hardwareMap);

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
                target = extender.RETRACTED;
            } else if (gamepad2.b) {
                target = extender.PICKUP;
            } else if (gamepad2.x) {
                target = extender.SCORE_RUNG;
            } else if (gamepad2.y) {
                target = extender.SCUIPA;
            } else if (gamepad2.dpad_left) {
                target = extender.MAX_LENGTH;
            } else if (gamepad2.start) {
                extender.resetEncoder();
            }

            extender.updateCoefficients(kP, kI, kD, kF);
            extender.setTarget(target);
            extender.runToTarget();

            telemetry.addData("Extender position:", extender.getPosition());
            telemetry.addData("Loop time:", loopTime.toString());
            telemetry.addData("Run time:", runTime.toString());
            telemetry.update();
        }
    }
}
