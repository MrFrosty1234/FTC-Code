package org.woen.OpModes.TestOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Flywheel {
    public static double ALPHA = 0.9;
    public static double FEED_FORWARD_MULTIPLY = 0.000429;
    public static double KP = 0.01;

    private final DcMotorEx flywheel;

    private double rawTPS = 0;
    private double filteredTPS = 0;

    public Flywheel(HardwareMap hardwareMap, String motorName, boolean reverse) {
        flywheel = hardwareMap.get(DcMotorEx.class, motorName);
        flywheel.setDirection(reverse ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);

        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(double targetTPS) {
        rawTPS = flywheel.getVelocity();
        filteredTPS = rawTPS * ALPHA + filteredTPS * (1 - ALPHA);

        double errorTPS = targetTPS - filteredTPS;
        double power = targetTPS * FEED_FORWARD_MULTIPLY + errorTPS * KP;

        flywheel.setPower(Math.max(-1.0, Math.min(1.0, power)));
    }

    public double getRawTPS() {
        return rawTPS;
    }

    public double getFilteredTPS() {
        return filteredTPS;
    }

    public void stop() {
        flywheel.setPower(0);
    }
}
