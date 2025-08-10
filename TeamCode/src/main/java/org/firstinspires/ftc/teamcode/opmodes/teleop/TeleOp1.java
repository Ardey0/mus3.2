package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Extender;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Rotate;
import org.firstinspires.ftc.teamcode.subsystems.Tilt;

import java.util.List;

public class TeleOp1 extends LinearOpMode {
    // Motoare sasiu
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx backRight = null;

    // Subsystems
    private Claw claw = null;
    private Rotate rotate = null;
    private Tilt tilt = null;
    private Extender extender = null;
    private Lift lift = null;
    private Limelight limelight = null;

    // Robot state enum
    private enum RobotState {
        NEUTRAL,
        SCORING_BASKET_HIGH,
        SCORING_BASKET_LOW,
        SCORE_BASKET_AND_RESET,
        SCORING_RUNG_HIGH,
        SCORING_RUNG_LOW,
        SCORE_RUNG_AND_RESET,
        COLLECTING_GROUND_HIGH,
        COLLECTING_GROUND_LOW,
        ALIGN_SAMPLE_HIGH,
        ALIGN_SAMPLE_LOW,
        COLLECT_GROUND_AND_RESET,
        COLLECTING_FENCE,
        COLLECT_FENCE_AND_RESET,
        EJECTING,
        EJECT_AND_RESET,
        ABORT
    }

    private RobotState robotState = null;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime loopTime = new ElapsedTime();
        ElapsedTime runTime = new ElapsedTime();

        // Creeaza gamepad-urile
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // Instantiaza motoarele de pe sasiu
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // TODO: Reverse la motoare

        // Instantiaza subsystems
        claw = new Claw(hardwareMap);
        rotate = new Rotate(hardwareMap);
        tilt = new Tilt(hardwareMap);
        extender = new Extender(hardwareMap);
        lift = new Lift(hardwareMap);
        limelight = new Limelight(hardwareMap);

        telemetry.setMsTransmissionInterval(250);

        // Set bulk read mode
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
            robotState = RobotState.NEUTRAL;
            runTime.reset();
        }

        while (opModeIsActive()) {
            loopTime.reset();

            // Bulk read si pe Control si pe Expansion Hub
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            // Store gamepad values from the previous loop iteration
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            // Store gamepad values from this loop iteration
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            switch (robotState) {
                case NEUTRAL:
                    break;
                case SCORING_BASKET_HIGH:
                    break;
                case SCORING_BASKET_LOW:
                    break;
                case SCORE_BASKET_AND_RESET:
                    break;
                case SCORING_RUNG_HIGH:
                    break;
                case SCORING_RUNG_LOW:
                    break;
                case SCORE_RUNG_AND_RESET:
                    break;
                case COLLECTING_GROUND_HIGH:
                    break;
                case COLLECTING_GROUND_LOW:
                    break;
                case ALIGN_SAMPLE_HIGH:
                    break;
                case ALIGN_SAMPLE_LOW:
                    break;
                case COLLECT_GROUND_AND_RESET:
                    break;
                case COLLECTING_FENCE:
                    break;
                case COLLECT_FENCE_AND_RESET:
                    break;
                case EJECTING:
                    break;
                case EJECT_AND_RESET:
                    break;
                case ABORT:
                    break;
                default:
                    telemetry.addLine("UNKNOWN ROBOT STATE");
                    break;
            }

            telemetry.addData("Robot state:", robotState);
            telemetry.addData("Loop time:", loopTime.toString());
            telemetry.addData("Run time:", runTime.toString());
            telemetry.update();
        }
    }
}
