package org.woen.Modules.DriveTrain;

import static com.qualcomm.robotcore.util.Range.clip;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.decrementExact;
import static java.lang.Math.max;
import static java.lang.Math.negateExact;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.woen.Modules.Odometery.Odometry;
import org.woen.Modules.Interface.RobotModule;
import org.woen.Robot.Robot;
import org.woen.Utility.PID.PID;

@Config
public class DriveTrain implements RobotModule {


    PID pidAngle = new PID(0, 0, 0, 0, 0, 0, 0); //TODO p,i,f
    PID pidY = new PID(0, 0, 0, 0, 0, 0, 0); //TODO p,i,f
    PID pidX = new PID(0, 0, 0, 0, 0, 0, 0); //TODO p,i,f

    Robot robot;

    DriveTrainMode mode = DriveTrainMode.AUTO;

    DcMotorEx rF;
    DcMotorEx lF;
    DcMotorEx rB;
    DcMotorEx lB;


    Odometry odometry;

    public DriveTrain(Robot robot) {
        this.robot = robot;
        odometry = robot.odometry;
    }


    @Override
    public void init() {
        rF = robot.devicePool.rMF;
        lF = robot.devicePool.lMF;
        rB = robot.devicePool.rMB;
        lB = robot.devicePool.lMB;

        rF.setDirection(DcMotorSimple.Direction.REVERSE);
        lF.setDirection(DcMotorSimple.Direction.FORWARD);
        rB.setDirection(DcMotorSimple.Direction.REVERSE);
        lB.setDirection(DcMotorSimple.Direction.FORWARD);

        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void stop() {
        rF.setPower(0);
        lF.setPower(0);
        rB.setPower(0);
        lB.setPower(0);
    }

    public void setPower(double x, double y, double h) {
        double lF = x + y + h;
        double rB = x + y - h;
        double lB = x - y + h;
        double rF = x - y - h;

        this.rB.setPower(clip(rB, -1, 1));
        this.lF.setPower(clip(lF, -1, 1));
        this.rF.setPower(clip(rF, -1, 1));
        this.lB.setPower(clip(lB, -1, 1));

    }


    ElapsedTime time = new ElapsedTime();

    public static double maxVel = 1;

    double x;
    double y;
    double h;

    private double errX = 0;
    private double errY = 0;
    private double errH = 0;

    private double getErrX() {
        return errX = x - odometry.getPosX();
    }

    private double getErrY() {
        return errY = y - odometry.getPosY();
    }

    private double getErrH() {
        errH = -atan2(errY, errX);
        if (errH > PI) {
            do {
                errH -= PI;
            } while (errH > PI);
        }
        if (errH < -PI) {
            do {
                errH += PI;
            } while (errH < -PI);
        }
        return errH;
    }

    boolean stop = true;


    public void setFieldPos(double x, double y, double h, double speed, boolean stop) {
        time.reset();

        this.x = x;
        this.y = y;
        this.h = h;
        this.stop = stop;

        maxVel = speed;
    }

    public void setFieldPos(double x, double y, double h) {
        time.reset();

        this.x = x;
        this.y = y;
        this.h = h;
        stop = true;

        maxVel = 1;
    }

    @Override
    public boolean isAtTarget() {
        return (x - odometry.getPosX()) > 5 && (y - odometry.getPosY()) > 5 && (h - odometry.getPosH()) > 5;
    }

    public void setMode(DriveTrainMode mod){
        mode = mod;
    }

    @Override
    public void update() {
        switch (mode) {
            case AUTO:
                if (!isAtTarget() && time.seconds() < 6.9) {
                    pidX.setTarget(x);

                    pidAngle.setTarget(h);

                    double uX = pidX.update(getErrX());
                    double uH = pidAngle.update(getErrH());
                    double uY = pidY.update(getErrY());



                    setPower(uX , uY, uH);
                }

                if (stop)
                    stop();
            case MANUAL:

        }
    }


}
