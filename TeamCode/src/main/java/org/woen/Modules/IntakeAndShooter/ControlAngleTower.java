package org.woen.Modules.IntakeAndShooter;
import com.qualcomm.robotcore.hardware.Servo;

import org.woen.MatchData.MatchData;
import org.woen.Modules.Interface.RobotModule;
import org.woen.Robot.Robot;

public class ControlAngleTower implements RobotModule {
    Robot robot;

    Servo towerServo;

    private static double startPose = ControlConst.startTowPosAss;

    public ControlAngleTower(Robot robot){
        this.robot = robot;
    }

    public void setStartPose(double setStartPose){
        startPose = setStartPose;
    }



    @Override
    public void init() {
        towerServo = robot.devicePool.towerAngleServo;
    }

    public void setPosition(double target){
      //  towerServo.setPosition(target);
    }


    @Override
    public void update() {

      /*  double angleToShoot = MatchData.goal.h - robot.odometry.getPosH();
        
        if (angleToShoot > 0.3 && angleToShoot < 0.7) {
            towerServo.setPosition(startPose + angleToShoot / ControlConst.ticksToTurn);
        }

       */
    }
}
