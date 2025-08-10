package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Extender;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.List;

public class CameraDebug extends LinearOpMode {
    public static int pipeline = 0;
    private Limelight limelight = null;
    public void runOpMode() throws InterruptedException {
        ElapsedTime loopTime = new ElapsedTime();
        ElapsedTime runTime = new ElapsedTime();

        limelight = new Limelight(hardwareMap);

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
                pipeline = limelight.RED_YELLOW;
                limelight.switchPipeline(pipeline);
            } else if (gamepad2.b) {
                pipeline = limelight.BLUE_YELLOW;
                limelight.switchPipeline(pipeline);
            } else if (gamepad2.x) {
                pipeline = limelight.RED;
                limelight.switchPipeline(pipeline);
            } else if (gamepad2.y) {
                pipeline = limelight.BLUE;
                limelight.switchPipeline(pipeline);
            } else if (gamepad2.dpad_left) {
                pipeline = limelight.YELLOW;
                limelight.switchPipeline(pipeline);
            } else if (gamepad2.dpad_right) {
                pipeline = limelight.RED_YELLOW_EXPERIMENTAL;
                limelight.switchPipeline(pipeline);
            }

            telemetry.addData("Tx:", limelight.getTargetTx());
            telemetry.addData("Ty:", limelight.getTargetTy());
            telemetry.addData("Area:", limelight.getTargetArea());
            telemetry.addData("Angle:", limelight.getAngle());
//            telemetry.addData("Color", limelight.getColor());
            telemetry.addData("Pipeline:", pipeline);
            telemetry.addData("Staleness:", limelight.getStaleness());
            telemetry.addData("Loop time:", loopTime.toString());
            telemetry.addData("Run time:", runTime.toString());
            telemetry.update();
        }
    }
}
