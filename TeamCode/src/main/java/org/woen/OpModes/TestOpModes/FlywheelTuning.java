package org.firstinspires.ftc.teamcode.evilULT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.woen.OpModes.TestOpModes.Flywheel;

@Config
@TeleOp
public class FlywheelTuning extends LinearOpMode {
    public static double TARGET_TPS = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Flywheel flywheel = new Flywheel(hardwareMap, "gun_motor_left", false);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Ready!");
        telemetry.update();

        DcMotor motor = hardwareMap.get(DcMotor.class, "motor_flow");
        DcMotor brush = hardwareMap.get(DcMotor.class, "motor_brush");

        waitForStart();
        telemetry.clear();

        while (opModeIsActive()) {
            flywheel.update(TARGET_TPS);
            motor.setPower(1);
            brush.setPower(1);
            telemetry.addData("TARGET_TPS", TARGET_TPS);
            telemetry.addData("rawTPS", flywheel.getRawTPS());
            telemetry.addData("filteredTPS", flywheel.getFilteredTPS());
            telemetry.update();
        }
        flywheel.stop();
    }
}



