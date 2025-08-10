package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Extender;
import org.firstinspires.ftc.teamcode.subsystems.Rotate;
import org.firstinspires.ftc.teamcode.subsystems.Tilt;

import java.util.List;

public class ServoDebug extends LinearOpMode {
    private Claw claw = null;
    private Rotate rotate = null;
    private Tilt tilt = null;
    public static double clawPosition = 0.5, rotatePosition = 0.5, tiltPosition = 0.5; /// ATENTIE SA NU RUPEM SERVOURILE/PRINTURILE

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime loopTime = new ElapsedTime();
        ElapsedTime runTime = new ElapsedTime();

        claw = new Claw(hardwareMap);
        rotate = new Rotate(hardwareMap);
        tilt = new Tilt(hardwareMap);

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

            /// ATENTIE SA NU RUPEM SERVOURILE/PRINTURILE
            claw.setPosition(clawPosition);
            rotate.setPosition(rotatePosition);
            tilt.setPosition(tiltPosition);

            telemetry.addData("Claw position:", claw.getPosition());
            telemetry.addData("Claw state:", claw.getState());
            telemetry.addData("Rotate position:", rotate.getPosition());
            telemetry.addData("Tilt position:", tilt.getPosition());
            telemetry.addData("Loop time:", loopTime.toString());
            telemetry.addData("Run time:", runTime.toString());
            telemetry.update();
        }
    }
}
