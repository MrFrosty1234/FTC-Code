package org.woen.Modules.Servo;

import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.Servo;

import org.woen.Modules.IntakeAndShooter.ControlConst;
import org.woen.Modules.Interface.RobotModule;
import org.woen.Robot.Robot;

public class ServoMovement implements RobotModule {



    private Servo angle;
    private Servo wall;
    private double anglePos;
    private double wallPos;


    public void wallOpen(){wallPos = ControlConst.openWall;}
    public void wallClose(){wallPos = ControlConst.closeWall;}
    public void anglePosNear(){wallPos = ControlConst.angleContNear;}
    public void anglePosFar(){wallPos = ControlConst.angleContFar;}



    Robot robot;

    public ServoMovement(Robot robot){
        this.robot = robot;
    }

    @Override
    public void init() {
        wall = robot.devicePool.wall;
        angle = robot.devicePool.angleServo;
        wallClose();
        anglePosNear();

    }

    @Override
    public void update() {
        wall.setPosition(wallPos);
        angle.setPosition(anglePos);
    }
}
