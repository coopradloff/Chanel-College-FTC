package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "SimpleTeleop Intake", group = "TeleOp")
public class Intake extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize motors matching the exact configuration names
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "left front");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "right front");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "left back");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "right back");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        // Enable active braking on all drivetrain motors and intake motor
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse right side motors so forward power drives all wheels forward
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Raw controller inputs (Y inverted so pushing stick up yields positive value)
            double rawY = -gamepad1.left_stick_y;
            double rawX = gamepad1.left_stick_x;
            double rawRx = gamepad1.right_stick_x;

            // Apply logarithmic throttle curve to joystick inputs
            double y = applyLogCurve(rawY);
            double x = applyLogCurve(rawX);
            double rx = applyLogCurve(rawRx);

            // Calculate drive motor powers
            double powerLF = y + x + rx;
            double powerRF = y - x - rx;
            double powerLB = y - x + rx;
            double powerRB = y + x - rx;

            // Normalize powers if any motor exceeds 1.0
            double max = Math.max(Math.abs(powerLF), 
                         Math.max(Math.abs(powerRF), 
                         Math.max(Math.abs(powerLB), Math.abs(powerRB))));

            if (max > 1.0) {
                powerLF /= max;
                powerRF /= max;
                powerLB /= max;
                powerRB /= max;
            }

            // Set drivetrain powers
            leftFront.setPower(powerLF);
            rightFront.setPower(powerRF);
            leftBack.setPower(powerLB);
            rightBack.setPower(powerRB);

            // Intake control using A for In (1.0), B for Out (-1.0)
            if (gamepad1.a) {
                intake.setPower(1.0);
            } else if (gamepad1.b || gamepad1.circle) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }

            // Telemetry output
            telemetry.addData("LF Power", powerLF);
            telemetry.addData("RF Power", powerRF);
            telemetry.addData("LB Power", powerLB);
            telemetry.addData("RB Power", powerRB);
            telemetry.update();
        }
    }

    // Applies logarithmic scaling to joystick inputs with a 5% deadband
    private double applyLogCurve(double input) {
        if (Math.abs(input) < 0.05) {
            return 0.0;
        }
        return Math.signum(input) * (Math.log10(1.0 + 9.0 * Math.abs(input)));
    }
}
