package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Extender;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Rotate;
import org.firstinspires.ftc.teamcode.subsystems.Tilt;

import java.util.List;

@TeleOp
@Config
public class TeleOp1 extends LinearOpMode {

    // Robot state enum
    private enum RobotState {
        NEUTRAL,
        SCORING_HIGH_BASKET,
        SCORING_LOW_BASKET,
        SCORE_BASKET_AND_RESET,
        SCORING_HIGH_RUNG,
        SCORING_LOW_RUNG,
        SCORE_RUNG_AND_RESET,
        COLLECTING_GROUND_HIGH,
        COLLECTING_GROUND_LOW,
        ALIGN_SAMPLE,
        COLLECT_GROUND_AND_RESET,
        COLLECTING_FENCE,
        COLLECT_FENCE_AND_RESET,
        EJECTING,
        EJECT_AND_RESET,
        ASCENDING,
        ASCENDED,
        ABORT
    }
    private RobotState robotState = null;
    private final double kP = 45, kI = 1, kD = 1.7, kF = 0; // TODO: Inapoi in private final
    private final double centerOfRobotToCameraCm = 26.5, pickupHighHeightCm = 32.5, pickupLowHeightCm = 19.4, cameraToClawCm = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        Translation2d frontLeftLocation =
                new Translation2d(0.162, 0.126);
        Translation2d frontRightLocation =
                new Translation2d(0.162, -0.126);
        Translation2d backLeftLocation =
                new Translation2d(-0.162, 0.126);
        Translation2d backRightLocation =
                new Translation2d(-0.162, -0.126);

        MecanumDriveKinematics kinematics = new MecanumDriveKinematics
                (
                        frontLeftLocation, frontRightLocation,
                        backLeftLocation, backRightLocation
                );

        int extenderManualCompensation = 0;
        double targetHeadingRad = 0, targetLengthCm = 0, xDegrees = 0, yDegrees = 0, xOffsetCm = 0, yOffsetCm = 0;

        ElapsedTime loopTime = new ElapsedTime();
        ElapsedTime runTime = new ElapsedTime();
        ElapsedTime sleepTimer = new ElapsedTime();

        // Creeaza gamepad-urile
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        PIDFController pidf = new PIDFController(kP, kI, kD, kF);

        // Initializeaza motoarele de pe sasiu
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initializeaza Pinpoint
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(80, 0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(37.251351252, DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

        // Initializeaza subsystems
        Claw claw = new Claw(hardwareMap);
        Rotate rotate = new Rotate(hardwareMap);
        Tilt tilt = new Tilt(hardwareMap);
        Extender extender = new Extender(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Limelight limelight = new Limelight(hardwareMap);

        telemetry.setMsTransmissionInterval(250);

        // Init position
        rotate.setPosition(Rotate.STRAIGHT);
        tilt.setPosition(Tilt.STRAIGHT);
        extender.setTarget(Extender.RETRACTED);
        tilt.setPosition(Tilt.UP);
        lift.setTarget(Lift.DOWN);
        claw.setPosition(Claw.CLOSED);
        limelight.switchPipeline(Limelight.RED_YELLOW_EXPERIMENTAL);

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

            pinpoint.update();

            lift.runToTarget();
            extender.runToTarget();

            pidf.setPIDF(kP, kI, kD, kF);

            switch (robotState) {
                case NEUTRAL:
                    /*
                     * Ca un mic exercitiu, hai sa nu facem nicio actiune in neutral.
                     * Hai doar sa asteptam input de la driver. Asta inseamna ca de fiecare data cand
                     * ne intoarcem in neutral trebuie deja sa avem comenzile date pentru ca robotul
                     * sa ajunga in starea "neutra".
                    */
                    if (currentGamepad2.a && !previousGamepad2.a)
                        robotState = RobotState.COLLECTING_GROUND_HIGH;
                    if (currentGamepad2.x && !previousGamepad2.x)
                        robotState = RobotState.COLLECTING_GROUND_LOW;
                    if (currentGamepad2.b && !previousGamepad2.b)
                        robotState = RobotState.COLLECTING_FENCE;
                    if (currentGamepad2.y && !previousGamepad2.y)
                        robotState = RobotState.EJECTING;
                    if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up)
                        robotState = RobotState.SCORING_HIGH_BASKET;
                    if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left)
                        robotState = RobotState.SCORING_LOW_BASKET;
                    if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down)
                        robotState = RobotState.SCORING_HIGH_RUNG;
                    if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right)
                        robotState = RobotState.SCORING_LOW_RUNG;
                    if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper)
                        robotState = RobotState.ASCENDING;
                    if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) { // Reset encoders
                        lift.resetEncoder();
                        extender.resetEncoder();
                    }
                    break;

                case SCORING_HIGH_BASKET:
                    lift.setTarget(Lift.HIGH_BASKET);
                    if (currentGamepad2.start && !previousGamepad2.start) {
                        robotState = RobotState.SCORE_BASKET_AND_RESET;
                        sleepTimer.reset();
                    }
                    break;

                case SCORING_LOW_BASKET:
                    lift.setTarget(Lift.LOW_BASKET);
                    if (currentGamepad2.start && !previousGamepad2.start) {
                        robotState = RobotState.SCORE_BASKET_AND_RESET;
                        sleepTimer.reset();
                    }
                    break;

                case SCORE_BASKET_AND_RESET:
                    tilt.setPosition(Tilt.STRAIGHT);
                    if (sleepTimer.milliseconds() >= 500) { // TODO: Teste alea alea
                        claw.setPosition(Claw.OPEN);
                        if (claw.getState() == Claw.State.OPEN) {
                            tilt.setPosition(Tilt.UP);
                            claw.setPosition(Claw.CLOSED);
                        }
                        lift.setTarget(Lift.DOWN);
                        robotState = RobotState.NEUTRAL;
                    }
                    break;

                case SCORING_HIGH_RUNG:
                    tilt.setPosition(Tilt.UP);
                    lift.setTarget(Lift.HIGH_RUNG);
                    if (currentGamepad2.start && !previousGamepad2.start) {
                        robotState = RobotState.SCORE_RUNG_AND_RESET;
                        sleepTimer.reset();
                    }
                    break;

                case SCORING_LOW_RUNG:
                    lift.setTarget(Lift.LOW_RUNG);
                    if (currentGamepad2.start && !previousGamepad2.start) {
                        robotState = RobotState.SCORE_RUNG_AND_RESET;
                        sleepTimer.reset();
                    }
                    break;

                case SCORE_RUNG_AND_RESET:
                    extender.setTarget(Extender.SCORE_RUNG);
                    if (extender.delta() < 30) {
                        claw.setPosition(Claw.OPEN);
                        if (claw.getState() == Claw.State.OPEN) {
                            extender.setTarget(Extender.RETRACTED);
                            lift.setTarget(Lift.DOWN);
                            claw.setPosition(Claw.CLOSED);
                            robotState = RobotState.NEUTRAL;
                        }
                    }
                    break;

                case COLLECTING_GROUND_HIGH:
                    lift.setTarget(Lift.PICKUP_HIGH);
                    rotate.setPosition(Rotate.STRAIGHT);
                    claw.setPosition(Claw.OPEN);
                    tilt.setPosition(Tilt.DOWN);
                    extender.setTarget(Extender.PICKUP + extenderManualCompensation);
                    if (gamepad2.left_trigger > 0.2)
                        extenderManualCompensation += 5;
                    if (gamepad2.right_trigger > 0.2)
                        extenderManualCompensation -= 5;

                    if (currentGamepad2.start && !previousGamepad2.start) {
                        xDegrees = limelight.getTargetTx();
                        yDegrees = limelight.getTargetTy();
                        xOffsetCm = Math.tan(Math.toRadians(xDegrees)) * pickupHighHeightCm;
                        yOffsetCm = Math.tan(Math.toRadians(yDegrees)) * pickupHighHeightCm + centerOfRobotToCameraCm + extender.getPosition() * 11.2 / 145.1;
                        targetHeadingRad = pinpoint.getHeading(AngleUnit.RADIANS) - Math.atan(xOffsetCm / yOffsetCm);
                        targetLengthCm = yOffsetCm - centerOfRobotToCameraCm - cameraToClawCm;
                        robotState = RobotState.ALIGN_SAMPLE;
                    }

                    telemetry.addData("Limelight tx:", limelight.getTargetTx());
                    telemetry.addData("Limelight ty:", limelight.getTargetTy());
                    break;

                case COLLECTING_GROUND_LOW:
                    lift.setTarget(Lift.PICKUP_LOW);
                    rotate.setPosition(Rotate.STRAIGHT);
                    claw.setPosition(Claw.OPEN);
                    tilt.setPosition(Tilt.DOWN);
                    extender.setTarget(Extender.PICKUP + extenderManualCompensation);
                    if (gamepad2.left_trigger > 0.2)
                        extenderManualCompensation += 5;
                    if (gamepad2.right_trigger > 0.2)
                        extenderManualCompensation -= 5;

                    if (currentGamepad2.start && !previousGamepad2.start) {
                        xDegrees = limelight.getTargetTx();
                        yDegrees = limelight.getTargetTy();
                        xOffsetCm = Math.tan(Math.toRadians(xDegrees)) * pickupHighHeightCm;
                        yOffsetCm = Math.tan(Math.toRadians(yDegrees)) * pickupHighHeightCm + centerOfRobotToCameraCm + extender.getPosition() * 11.2 / 145.1;
                        targetHeadingRad = pinpoint.getHeading(AngleUnit.RADIANS) - Math.atan(xOffsetCm / yOffsetCm);
                        targetLengthCm = yOffsetCm - centerOfRobotToCameraCm - 3;
                        robotState = RobotState.ALIGN_SAMPLE;
                    }
                    
                    telemetry.addData("Limelight tx:", limelight.getTargetTx());
                    telemetry.addData("Limelight ty:", limelight.getTargetTy());
                    break;

                case ALIGN_SAMPLE:
                    double currentHeading = pinpoint.getHeading(AngleUnit.RADIANS);
                    double error = targetHeadingRad - currentHeading;
                    extender.setTarget((int)(targetLengthCm / 11.2 * 145.1));  // Conversia in ticks + cast la int

                    telemetry.addData("Heading error", Math.toDegrees(error));
                    telemetry.addData("Target heading", Math.toDegrees(targetHeadingRad));
                    telemetry.addData("Target length cm", targetLengthCm);
                    telemetry.addData("Target length ticks", targetLengthCm / 11.2 * 145.1);
                    telemetry.addData("Extender error", extender.delta());
                    if (Math.abs(error) > 0.02 || extender.delta() > 2) {
                        double robotAngularVelocity = pidf.calculate(currentHeading, targetHeadingRad);
                        ChassisSpeeds robotSpeeds = new ChassisSpeeds(0, 0, robotAngularVelocity);
                        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(robotSpeeds);
                        frontLeft.setPower(wheelSpeeds.frontLeftMetersPerSecond);
                        backLeft.setPower(wheelSpeeds.rearLeftMetersPerSecond);
                        frontRight.setPower(wheelSpeeds.frontRightMetersPerSecond);
                        backRight.setPower(wheelSpeeds.rearRightMetersPerSecond);
                    } else {
                        rotate.trackTarget(limelight);
                        robotState = RobotState.COLLECT_GROUND_AND_RESET;
                    }
                    break;

                case COLLECT_GROUND_AND_RESET:
                    lift.setTarget(Lift.DOWN);
                    if (lift.delta() < 5) {
                        claw.setPosition(Claw.CLOSED);
                        if (claw.getState() == Claw.State.CLOSED_WITH_SAMPLE) {
                            tilt.setPosition(Tilt.UP);
                            rotate.setPosition(Rotate.STRAIGHT);
                            extender.setTarget(Extender.RETRACTED);
                            robotState = RobotState.NEUTRAL;
                        }
                    }
                    telemetry.addData("Claw state", claw.getState());
                    telemetry.addData("Claw encoder voltage", claw.getPosition());
                    break;

                case COLLECTING_FENCE:
                    lift.setTarget(Lift.PICKUP_FENCE);
                    claw.setPosition(Claw.OPEN);
                    tilt.setPosition(Tilt.STRAIGHT);
                    if (currentGamepad2.start && !previousGamepad2.start) {
                        robotState = RobotState.COLLECT_FENCE_AND_RESET;
                    }
                    break;

                case COLLECT_FENCE_AND_RESET:
                    claw.setPosition(Claw.CLOSED);
                    if (claw.getState() == Claw.State.CLOSED_WITH_SAMPLE) {
                        lift.setTarget(Lift.PICKUP_FENCE + 100);
                        tilt.setPosition(Tilt.UP);
                        if (lift.getPosition() == Lift.PICKUP_FENCE + 100) {
                            lift.setTarget(Lift.DOWN);
                            robotState = RobotState.NEUTRAL;
                        }
                    } else if (claw.getState() == Claw.State.CLOSED_NO_SAMPLE) {
                        robotState = RobotState.COLLECTING_FENCE;
                    }
                    break;

                case EJECTING:
                    extender.setTarget(Extender.PICKUP);
                    tilt.setPosition(Tilt.DOWN);
                    if (extender.delta() < 10) {
                        robotState = RobotState.EJECT_AND_RESET;
                    }
                    break;

                case EJECT_AND_RESET:
                    claw.setPosition(Claw.OPEN);
                    if (claw.getState() == Claw.State.OPEN) {
                        extender.setTarget(Extender.RETRACTED);
                        tilt.setPosition(Tilt.UP);
                        claw.setPosition(Claw.CLOSED);
                        robotState = RobotState.NEUTRAL;
                    }
                    break;

                case ASCENDING:
                    lift.setTarget(2750);
                    if (lift.delta() < 20) {
                        extender.setTarget(Extender.MAX_LENGTH);
                        if (currentGamepad2.start && !previousGamepad2.start) {
                            robotState = RobotState.ASCENDED;
                        }
                    }
                    break;

                case ASCENDED:
                    lift.setTarget(1900);
                    break;

                case ABORT:
                    rotate.setPosition(Rotate.STRAIGHT);
                    tilt.setPosition(Tilt.STRAIGHT);
                    extender.setTarget(Extender.RETRACTED);
                    tilt.setPosition(Tilt.UP);
                    lift.setTarget(Lift.DOWN);
                    claw.setPosition(Claw.CLOSED);
                    robotState = RobotState.NEUTRAL;
                    break;

                default:
                    telemetry.addLine("UNKNOWN ROBOT STATE");
                    break;
            }

            if (currentGamepad2.back && !previousGamepad2.back) {
                robotState = RobotState.ABORT;
            }

            telemetry.addData("Robot state:", robotState);
            telemetry.addData("Lift position:", lift.getPosition());
            telemetry.addData("Extender position:", extender.getPosition());
            telemetry.addData("Heading:", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Loop time:", loopTime.toString());
            telemetry.addData("Run time:", runTime.toString());
            telemetry.update();
        }
    }
}
