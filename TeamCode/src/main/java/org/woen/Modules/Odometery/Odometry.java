package org.woen.Modules.Odometery;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.R;
import org.woen.MatchData.MatchData;
import org.woen.Math.ExponentialFilter;
import org.woen.Modules.Interface.RobotModule;
import org.woen.Robot.Robot;



public class Odometry implements RobotModule {


    Robot robot;
    public GoBildaPinpointDriver pinpoint;


    public Odometry(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        pinpoint = robot.devicePool.pinpoint;
        pinpoint.setOffsets(110, 60, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

    }

    public void reset(){
        robot.devicePool.pinpoint.resetPosAndIMU();
    }

    public void setPosition(double x, double y, double h){
        robot.devicePool.pinpoint.setPosition(new Pose2D(CM, x, y, AngleUnit.DEGREES, h));
    }

    public double getPosX(){
        return robot.devicePool.pinpoint.getPosition().getX(CM);
    }

    public double getVelX(){
        return robot.devicePool.pinpoint.getVelX(CM);
    }
    public double getVelY(){
        return robot.devicePool.pinpoint.getVelY(CM);
    }

    public double getPosY(){
        return robot.devicePool.pinpoint.getPosition().getY(CM);
    }

    public double getPosH(){
        return robot.devicePool.pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }

    @Override
    public void update(){
        robot.devicePool.pinpoint.update();
    }
}


