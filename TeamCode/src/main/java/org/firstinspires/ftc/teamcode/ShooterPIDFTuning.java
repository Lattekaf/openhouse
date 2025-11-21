package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shooter PIDF Tuning", group = "Tuning")
public class ShooterPIDFTuning extends LinearOpMode {

    // -------- TODO: ENTER YOUR MOTOR NAMES --------
    private static final String MOTOR_LEFT = "SL";
    private static final String MOTOR_RIGHT = "SR";

    // -------- PIDF COEFFICIENTS (edit these) --------
    // Start simple: increase P until stable, add D for damping, I for offset, F for feed-forward
    private static final double P = 6.0;
    private static final double I = 0.0;
    private static final double D = 0.02;
    private static final double F = 0.0;

    // Shooter target
    private static final double TARGET_RPM = 3800;

    // Motor info (update ticks per rev for your motor)
    private static final double TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, MOTOR_LEFT);
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, MOTOR_RIGHT);
        Servo Ag;

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);

        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
        left.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        right.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        telemetry.addLine("Shooter PIDF Tuning Ready");
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double targetTPS = rpmToTPS(TARGET_RPM);

            left.setVelocity(targetTPS);
            right.setVelocity(targetTPS);

            double rpmL = getRPM(left);
            double rpmR = getRPM(right);
            double avgRPM = (rpmL + rpmR) / 2.0;

            telemetry.addLine("=== SHOOTER PIDF TUNING ===");
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Left RPM", rpmL);
            telemetry.addData("Right RPM", rpmR);
            telemetry.addData("Avg RPM", avgRPM);
            telemetry.addData("Error", TARGET_RPM - avgRPM);

            telemetry.addLine();
            telemetry.addData("Left TPS", left.getVelocity());
            telemetry.addData("Right TPS", right.getVelocity());
            telemetry.addLine();
            telemetry.addData("P", P);
            telemetry.addData("I", I);
            telemetry.addData("D", D);
            telemetry.addData("F", F);

            telemetry.update();
        }
    }

    private double getRPM(DcMotorEx motor) {
        double tps = motor.getVelocity();        // ticks per second
        return tps * 60.0 / TICKS_PER_REV / GEAR_RATIO;
    }

    private double rpmToTPS(double rpm) {
        return rpm * TICKS_PER_REV / 60.0 * GEAR_RATIO;
    }
}
