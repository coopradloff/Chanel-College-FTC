package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "SimpleTeleop Intake v2", group = "TeleOp")
public class Intake_v2 extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront, rightFront, leftBack, rightBack, intake;

    // Slow-mode state (toggled by right bumper)
    private boolean slowMode = false;
    private boolean lastRightBumper = false;
    private static final double SLOW_MODE_MULTIPLIER = 0.4;

    // Intake hold mode (toggled by left bumper) - keeps intake running without holding trigger
    private boolean intakeHoldActive = false;
    private boolean lastLeftBumper = false;

    // Field-centric toggle (PS button / options) - optional if you add an IMU later
    private boolean fieldCentric = false;
    private boolean lastOptionsButton = false;

    // Rumble feedback so you can feel state changes without watching the screen
    private ElapsedTime rumbleCooldown = new ElapsedTime();

    // Loop timing (useful for diagnosing lag)
    private ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            handleDrive();
            handleIntake();
            handleToggles();
            updateTelemetry();
        }
    }

    private void initHardware() {
        leftFront = hardwareMap.get(DcMotor.class, "left front");
        rightFront = hardwareMap.get(DcMotor.class, "right front");
        leftBack = hardwareMap.get(DcMotor.class, "left back");
        rightBack = hardwareMap.get(DcMotor.class, "right back");
        intake = hardwareMap.get(DcMotor.class, "intake");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
    }

    private void handleDrive() {
        double rawY = -gamepad1.left_stick_y;
        double rawX = gamepad1.left_stick_x;
        double rawRx = gamepad1.right_stick_x;

        double y = applyThrottleCurve(rawY);
        double x = applyThrottleCurve(rawX);
        double rx = applyThrottleCurve(rawRx);

        double powerLF = y + x + rx;
        double powerRF = y - x - rx;
        double powerLB = y - x + rx;
        double powerRB = y + x - rx;

        double max = Math.max(Math.abs(powerLF),
                     Math.max(Math.abs(powerRF),
                     Math.max(Math.abs(powerLB), Math.abs(powerRB))));

        if (max > 1.0) {
            powerLF /= max;
            powerRF /= max;
            powerLB /= max;
            powerRB /= max;
        }

        if (slowMode) {
            powerLF *= SLOW_MODE_MULTIPLIER;
            powerRF *= SLOW_MODE_MULTIPLIER;
            powerLB *= SLOW_MODE_MULTIPLIER;
            powerRB *= SLOW_MODE_MULTIPLIER;
        }

        leftFront.setPower(powerLF);
        rightFront.setPower(powerRF);
        leftBack.setPower(powerLB);
        rightBack.setPower(powerRB);
    }

    private void handleIntake() {
        double intakeIn = gamepad1.right_trigger;
        double intakeOut = gamepad1.left_trigger;

        if (intakeHoldActive) {
            // Hold mode overrides triggers, but left trigger can still reverse to clear a jam
            if (intakeOut > 0.05) {
                intake.setPower(-intakeOut);
            } else {
                intake.setPower(1.0);
            }
        } else if (intakeIn > 0.05) {
            intake.setPower(intakeIn);
        } else if (intakeOut > 0.05) {
            intake.setPower(-intakeOut);
        } else if (gamepad1.a) {
            intake.setPower(1.0);
        } else if (gamepad1.circle) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }
    }

    private void handleToggles() {
        // Slow mode: right bumper
        boolean rb = gamepad1.right_bumper;
        if (rb && !lastRightBumper) {
            slowMode = !slowMode;
            rumble(slowMode ? 1 : 2); // different pulse counts = different feedback
        }
        lastRightBumper = rb;

        // Intake hold: left bumper
        boolean lb = gamepad1.left_bumper;
        if (lb && !lastLeftBumper) {
            intakeHoldActive = !intakeHoldActive;
            rumble(1);
        }
        lastLeftBumper = lb;

        // Reserved toggle: options/share button (e.g. future field-centric mode once an IMU is wired up)
        boolean options = gamepad1.options;
        if (options && !lastOptionsButton) {
            fieldCentric = !fieldCentric;
        }
        lastOptionsButton = options;

        // Square = emergency stop everything instantly (useful if a driver panics mid-match)
        if (gamepad1.square) {
            stopAllMotors();
        }
    }

    private void stopAllMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        intake.setPower(0);
        intakeHoldActive = false;
    }

    // Simple non-blocking rumble helper; pulses gamepad1 for driver feedback on toggles
    private void rumble(int pulses) {
        if (rumbleCooldown.milliseconds() < 150) return; // debounce so it doesn't spam
        gamepad1.rumble(pulses);
        rumbleCooldown.reset();
    }

    private void updateTelemetry() {
        telemetry.addData("Loop Time (ms)", "%.1f", loopTimer.milliseconds());
        loopTimer.reset();
        telemetry.addData("Slow Mode", slowMode ? "ON" : "OFF");
        telemetry.addData("Intake Hold", intakeHoldActive ? "ON" : "OFF");
        telemetry.addData("Field Centric (reserved)", fieldCentric ? "ON" : "OFF");
        telemetry.addData("LF Power", "%.2f", leftFront.getPower());
        telemetry.addData("RF Power", "%.2f", rightFront.getPower());
        telemetry.addData("LB Power", "%.2f", leftBack.getPower());
        telemetry.addData("RB Power", "%.2f", rightBack.getPower());
        telemetry.addData("Intake Power", "%.2f", intake.getPower());
        telemetry.update();
    }

    // Power curve: 85% stick position maps to ~40% output power (0.85^5.64 ≈ 0.40)
    private double applyThrottleCurve(double input) {
        double absInput = Math.abs(input);
        if (absInput < 0.05) {
            return 0.0; // deadband
        }
        absInput = Math.min(absInput, 1.0);
        double scaledPower = Math.pow(absInput, 5.64);
        return Math.signum(input) * scaledPower;
    }
}
