package org.woen.Modules.DriveTrain;

import static com.qualcomm.robotcore.util.Range.clip;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.decrementExact;
import static java.lang.Math.max;
import static java.lang.Math.negateExact;
import static java.lang.Math.signum;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.woen.Modules.Odometery.Odometry;
import org.woen.Modules.Interface.RobotModule;
import org.woen.Robot.Robot;
import org.woen.Utility.PID.PID;

@Config
public class DriveTrain implements RobotModule {


    public static double pX = 0.0085;
    public static double dX = 0.001;
    public static double iX = 0;

    public static double pH = 0.0085;
    public static double dH = 0.001;
    public static double iH = 0;

    PID pidAngle = new PID(pH,iH , dH, 0, 0, 0, 0); //TODO p,i,f
    PID pidY = new PID(0.0085, 0.001, 0, 0, 0, 0, 0); //TODO p,i,f
    PID pidX = new PID(pX, iX, dX, 0, 0, 0, 0); //TODO p,i,f

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
        lF = robot.devicePool.rMF;
        rF = robot.devicePool.lMF;
        lB = robot.devicePool.rMB;
        rB = robot.devicePool.lMB;

        rF.setDirection(DcMotorSimple.Direction.FORWARD);
        lF.setDirection(DcMotorSimple.Direction.REVERSE);
        rB.setDirection(DcMotorSimple.Direction.FORWARD);
        lB.setDirection(DcMotorSimple.Direction.REVERSE);

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

    public void setPower(double x, double y, double h, double den) {
        double lF = (x + y - h) / den;
        double rB = (x + y + h) / den;
        double lB = (x - y - h) / den;
        double rF = (x - y + h) / den;

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

    public void fieldMovement(double x, double y, double h){
        pidX.reset();
        pidY.reset();
        pidAngle.reset();

        double errX = x - odometry.getPosX();
        double errY = y - odometry.getPosY();
        double errH = h - odometry.getPosH();

        while (abs(errH) > 180) {
            errH -= 360 * signum(errH);
        }


        pidX.update(errX);
        pidY.update(errY);
        pidAngle.update(errH);

        ElapsedTime elapsedTime = new ElapsedTime();
        time.reset();

        FtcDashboard.getInstance().getTelemetry().addLine("In cycle");
        FtcDashboard.getInstance().getTelemetry().update();

        while((abs(errX) > 5 || abs(errY) > 5 || abs(errH) > 5) && time.seconds() < 5 &&  robot.linearOpMode.opModeIsActive()){

             errX = x - odometry.getPosX();
             errY = y - odometry.getPosY();
             errH = h - odometry.getPosH();

            FtcDashboard.getInstance().getTelemetry().addData("isItTarget", isAtTarget(x,y,h));

            while (abs(errH) > 180) {
                errH -= 360 * signum(errH);
            }

            robot.update();

            FtcDashboard.getInstance().getTelemetry().addData("target", y);
            FtcDashboard.getInstance().getTelemetry().addData("pose", odometry.getPosY());
            FtcDashboard.getInstance().getTelemetry().update();

            double powerX = pidX.update(errX);
            double powerY = pidY.update(errY);
            double powerH = pidAngle.update(errH);

            setPower(-powerX, powerY, powerH);
        }


        FtcDashboard.getInstance().getTelemetry().addLine("Out cycle");
        FtcDashboard.getInstance().getTelemetry().update();
        stop();

    }

    public boolean isAtTarget(double x, double y, double h) {
        return abs(x - odometry.getPosX()) > 5 || abs(y - odometry.getPosY()) > 5 || abs(h - odometry.getPosH()) > 5;
    }

    public void setMode(DriveTrainMode mod){
        mode = mod;
    }

    @Override
    public void update() {

    }


}
