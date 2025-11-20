package org.woen.Modules.Servo;

import com.qualcomm.robotcore.hardware.Servo;

import org.woen.Modules.Interface.RobotModule;
import org.woen.Robot.Robot;

public class ServoMovement implements RobotModule {


    public Servo cEatServo;
    public Servo lEatServo;
    public Servo rEatServo;
    public Servo rAngleServo;
    public Servo cAngleServo;
    public Servo lAngleServo;
    public Servo wall;

    public double cServoPos;
    public double lServoPos;
    public double rServoPos;
    public double rServoAnglePos;
    public double cServoAnglePos;
    public double lServoAnglePos;
    public double wallPos;


    public void rShoot(){
        rServoPos = Config.eatShotR;
    }
    public void lShoot(){
        lServoPos = Config.eatShotL;
    }
    public void cShoot(){
        cServoPos = Config.eatShotC;
    }
    public void rEat(){
        rServoPos = Config.eatRLow;
    }
    public void cEat(){
        cServoPos = Config.eatCLow;
    }
    public void lEat(){
        lServoPos = Config.eatLLow;
    }
    public void rAngleFar(){rServoAnglePos = Config.angleContollerRFar;}
    public void lAngleFar(){lServoAnglePos = Config.angleContollerLFar;}
    public void cAngleFar(){cServoAnglePos = Config.angleContollerCFar;}
    public void rAngleNear(){rServoAnglePos = Config.angleContollerCNear;}
    public void lAngleNear(){lServoAnglePos = Config.angleContollerCNear;}
    public void cAngleNear(){cServoAnglePos = Config.angleContollerCNear;}
    public void wallOpen(){wallPos = Config.openWall;}
    public void wallClose(){wallPos = Config.closeWall;}

    public void allEat(){
        rEat();
        lEat();
        cEat();
    }
    public void allShoot(){
        rShoot();
        lShoot();
        cShoot();
    }
    public void allAngleNear(){
        rAngleNear();
        lAngleNear();
        cAngleNear();
    }
    public void allAngleFar(){
        rAngleFar();
        lAngleFar();
        cAngleFar();
    }


    Robot robot;

    public ServoMovement(Robot robot){
        this.robot = robot;
    }

    @Override
    public void init() {
        cEatServo = robot.devicePool.cEatServo;
        rEatServo = robot.devicePool.rEatServo;
        lEatServo = robot.devicePool.lEatServo;

        cAngleServo = robot.devicePool.cEatServo;
        rAngleServo = robot.devicePool.rAngleServo;
        lAngleServo = robot.devicePool.lAngleServo;

        wall = robot.devicePool.wall;
        allEat();
        wallOpen();
        allAngleNear();
    }

    @Override
    public void update() {
        wall.setPosition(cServoPos);

        cAngleServo.setPosition(cServoAnglePos);
        rAngleServo.setPosition(rServoAnglePos);
        lAngleServo.setPosition(lServoAnglePos);

        rEatServo.setPosition(rServoPos);
        cEatServo.setPosition(cServoPos);
        lEatServo.setPosition(lServoPos);
    }
}
