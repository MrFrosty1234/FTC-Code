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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.woen.Modules.Odometery.Odometry;
import org.woen.Modules.Interface.RobotModule;
import org.woen.Robot.Robot;
import org.woen.Utility.PID.PID;

@Config
public class DriveTrain implements RobotModule {


    PID pidAngle = new PID(0,0,0,0,0,0,0); //TODO p,i,f
    PID pidY = new PID(0,0,0,0,0,0,0); //TODO p,i,f
    PID pidX = new PID(0,0,0,0,0,0,0); //TODO p,i,f
    PID pidVel = new PID(0,0,0,0,0,0,0); //TODO  p,d

    Robot robot;

    DcMotorEx r;
    DcMotorEx l;

    Odometry odometry;

    public DriveTrain(Robot robot){
        this.robot = robot;
        odometry = robot.odometry;
    }


    @Override
    public void init(){
        r = robot.devicePool.rM;

        l = robot.devicePool.lM;


        l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        reset();
    }

    public void reset(){
        l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        r.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void stop(){
        r.setPower(0);
        l.setPower(0);
    }

    public void setPower(double x, double h){
        double l = x + h;
        double r = x - h;

        this.r.setPower(clip(r, -1, 1));
        this.l.setPower(clip(l, -1, 1));

    }


    ElapsedTime time = new ElapsedTime();

    public static double maxVel = 1;

    double x ;
    double y ;
    double h ;

    private double errX = 0;
    private double errY = 0;
    private double errH = 0;

    private double getErrX(){
        return errX = x - odometry.getPos().x;
    }

    private double getErrY(){
        return errY = y - odometry.getPos().y;
    }
    private double getErrH(){
        errH = -atan2(errY,  errX);
        if(errH > PI){
            do{
                errH -= PI;
            }while(errH > PI);
        }
       if(errH < -PI){
           do{
               errH += PI;
           }while (errH < - PI);
       }
       return errH;
    }

    boolean stop = true;


    public void setFieldPos(double x, double y, double h, double speed, boolean stop){
        time.reset();

        this.x = x;
        this.y = y;
        this.h = h;
        this.stop = stop;

        maxVel = speed;
    }
    public void setFieldPos(double x, double y, double h){
        time.reset();

        this.x = x;
        this.y = y;
        this.h = h;
        stop = true;

        maxVel = 1;
    }

    @Override
    public boolean isAtTarget(){
        return (x - odometry.getPos().x) > 5 && (y - odometry.getPos().y) > 5 && (h - odometry.getPos().h) > 5;
    }

    @Override
    public void update(){

        if(!isAtTarget() && time.seconds() < 6.9){
            pidX.setTarget(x);

            pidAngle.setTarget(h);

            double uX = pidX.update(getErrX());
            double uH = pidAngle.update(getErrH());

            pidVel.setTarget(maxVel);

            double uVelX = pidVel.update(maxVel - (odometry.getRealVelLeft() + odometry.getRealVelRight()) / 2);


            double uVelH = pidVel.update(maxVel - odometry.getRealVelHeading());

            setPower(uX + uVelX, uH + uVelH);
        }

        if(stop)
            stop();

    }


}
