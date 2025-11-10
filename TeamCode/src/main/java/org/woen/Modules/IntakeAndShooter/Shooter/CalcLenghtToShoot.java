package org.woen.Modules.IntakeAndShooter.Shooter;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import org.woen.MatchData.MatchData;
import org.woen.Math.Pose2D;
import org.woen.Pools.PositionPool.Positions;
import org.woen.Robot.Robot;
import org.woen.Utility.Team.TEAM;

public class CalcLenghtToShoot {

    Robot robot;

    private Pose2D startPos = MatchData.startPos;

    private Pose2D goal;

    public static double heightMax = 95.0;

    private double g = 9.81;

    private double l = 0;

    private double power = 1;

    double angle = 30;

    public double getPower() {
        return power;
    }

    public CalcLenghtToShoot(Robot robot){
        this.robot = robot;
    }


    public void init(){
        if(MatchData.team == TEAM.RED)
            goal = Positions.redGoal;
        else goal = Positions.blueGoal;

    }

    public void update(){

        Pose2D robotPos = robot.odometry.getPos().minusPos(startPos);

        l = robotPos.minusPos(goal).x;

        power = sqrt(l * g / (sin(angle) * cos(angle)));

    }



}
