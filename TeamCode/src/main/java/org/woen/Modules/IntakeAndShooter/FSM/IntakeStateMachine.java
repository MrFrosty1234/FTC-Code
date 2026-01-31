package org.woen.Modules.IntakeAndShooter.FSM;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.woen.Modules.IntakeAndShooter.FSM_STATES;
import org.woen.Modules.Interface.RobotModule;
import org.woen.Robot.Robot;

public class IntakeStateMachine implements RobotModule {

    FSM_STATES state = FSM_STATES.EAT;

    FSM_STATES targetState = FSM_STATES.EAT;

    Robot robot;
    //max is pidor ebany
    //suck my dick my boy
    // maxim huesos tupoy
    //msxim ne umeet igrat v basketball on ne negr
    DcMotorEx lGun;

    DcMotorEx rGun;

    public IntakeStateMachine(Robot robot) {
        this.robot = robot;
    }


    public void setState(FSM_STATES state) {
        this.state = state;
    }

    @Override
    public void init() {
        robot.servoMovement.init();
        lGun = robot.devicePool.lShooter;
        rGun = robot.devicePool.rShooter;

        lGun.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rGun.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lGun.setDirection(DcMotorSimple.Direction.FORWARD);
        rGun.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    ElapsedTime time = new ElapsedTime();

    public void updateStates() {
        switch (state) {
            case EAT:
                targetState = FSM_STATES.EAT;
                rGun.setPower(0.5);
                lGun.setPower(0.5);

                robot.servoMovement.wallClose();
                robot.servoMovement.allAngleNear();
                robot.servoMovement.allEat();
                break;
            case SHOOT_FAR:
                targetState = FSM_STATES.EAT;
                rGun.setPower(1);
                lGun.setPower(1);

                if (time.seconds() > 0.5) {
                    robot.servoMovement.allAngleFar();
                    robot.servoMovement.allShoot();
                }
                if (time.seconds() > 0.7) {
                    robot.servoMovement.wallOpen();
                }
                setState(FSM_STATES.EAT);
                break;
            case SHOOT_NEAR:
                targetState = FSM_STATES.EAT;
                rGun.setPower(1);
                lGun.setPower(1);

                if (time.seconds() > 0.5) {
                    robot.servoMovement.allAngleNear();
                    robot.servoMovement.allShoot();
                }
                if (time.seconds() > 0.7) {
                    robot.servoMovement.wallOpen();
                }
                setState(FSM_STATES.EAT);
                break;
            case REVERSE_BRUSHES:
                targetState = FSM_STATES.REVERSE_BRUSHES;
                rGun.setPower(-1);
                lGun.setPower(-1);

                robot.servoMovement.allAngleNear();
                robot.servoMovement.allShoot();
                robot.servoMovement.wallClose();

                break;
        }
    }


    @Override
    public void update() {
        if(state == targetState)
            time.reset();
        updateStates();
    }
}
