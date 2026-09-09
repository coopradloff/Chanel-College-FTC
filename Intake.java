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

        // Reverse right side motors so forward power drives all wheels forward
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Controller inputs (Y is inverted so pushing stick up yields positive value)
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Speed multiplier: 1.0x when Right Bumper is pressed, 0.6x otherwise
            double multiplier = gamepad1.right_bumper ? 1.0 : 0.6;

            // Calculate drive motor powers
            double powerLF = (y + x + rx) * multiplier;
            double powerRF = (y - x - rx) * multiplier;
            double powerLB = (y - x + rx) * multiplier;
            double powerRB = (y + x - rx) * multiplier;

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

            // Intake control: X for In (1.0), O / Circle / B for Out (-1.0)
            if (gamepad1.x) {
                intake.setPower(1.0);
            } else if (gamepad1.b || gamepad1.circle) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }

            // Telemetry output
            telemetry.addData("Speed", multiplier == 1.0 ? "100%" : "50%");
            telemetry.addData("LF Power", powerLF);
            telemetry.addData("RF Power", powerRF);
            telemetry.addData("LB Power", powerLB);
            telemetry.addData("RB Power", powerRB);
            telemetry.update();
        }
    }
}
