package org.woen.Modules.IntakeAndShooter.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.woen.Modules.Interface.RobotModule;
import org.woen.Robot.Robot;

public class Shooter implements RobotModule {

    DcMotorEx shooter;

    Robot robot;

    private double rawTPS = 0;
    private double filteredTPS = 0;

    public static double ALPHA = 0.9;
    public static double FEED_FORWARD_MULTIPLY = 0.001;
    public static double KP = 0.1;

    public static double targetTPS = 0;

    public Shooter(Robot robot){
        this.robot = robot;
    }

    @Override
    public void init() {
        shooter = robot.devicePool.shooterMotor;

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void setTarget(double target){
      targetTPS = target;
    }

    public double getRawTPS() {
        return rawTPS;
    }

    public double getFilteredTPS() {
        return filteredTPS;
    }

    public void stop() {
        shooter.setPower(0);
    }


    @Override
    public void update() {
        rawTPS = shooter.getVelocity();
        filteredTPS = rawTPS * ALPHA + filteredTPS * (1 - ALPHA);

        double errorTPS = targetTPS - filteredTPS;
        double power = targetTPS * FEED_FORWARD_MULTIPLY + errorTPS * KP;

        shooter.setPower(Math.max(-1.0, Math.min(1.0, power)));

    }
}
