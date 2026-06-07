package org.woen.Modules.IntakeAndShooter.FSM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.woen.Modules.IntakeAndShooter.ControlConst;
import org.woen.Modules.IntakeAndShooter.FSM_STATES;
import org.woen.Modules.Interface.RobotModule;
import org.woen.Robot.Robot;

public class IntakeStateMachine implements RobotModule {

    public FSM_STATES state = FSM_STATES.EAT;

    public FSM_STATES targetState = FSM_STATES.EAT;

    Robot robot;


    //max is pidor ebany
    //suck my dick my boy
    // maxim huesos tupoy
    //msxim ne umeet igrat v basketball on ne negr

    DcMotorEx flow;

    DcMotorEx brush;

    public IntakeStateMachine(Robot robot) {
        this.robot = robot;
    }


    public void setState(FSM_STATES state) {
        this.state = state;
    }

    @Override
    public void init() {
        robot.servoMovement.init();
        robot.shooter.init();
        flow = robot.devicePool.flowMotor;
        brush = robot.devicePool.brush;

        flow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brush.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flow.setDirection(DcMotorSimple.Direction.FORWARD);
        brush.setDirection(DcMotorSimple.Direction.FORWARD);
        time.reset();
        timerToFlow.reset();
    }


    ElapsedTime time = new ElapsedTime();

    ElapsedTime timerToFlow = new ElapsedTime();

    private int countCycle = 0;

    public void updateStates() {
        switch (state) {
            case EAT:
                targetState = FSM_STATES.EAT;
                robot.devicePool.wall.setPosition(1);
                robot.servoMovement.anglePosNear();
                robot.devicePool.angleServo.setPosition(0.5);
                flow.setPower(0.55);
               /* if(timerToFlow.milliseconds() < 200) {
                    flow.setPower(0.4);
                }
                if(timerToFlow.milliseconds() > 150 && timerToFlow.milliseconds() < 200){
                    flow.setPower(-0.4);
                }
                if(timerToFlow.milliseconds() > 200)
                    timerToFlow.reset();

                */
                brush.setPower(1);
                robot.shooter.setTarget(ControlConst.nearVel);
                break;
            case SHOOT_NEAR:
                targetState = FSM_STATES.EAT;
                robot.devicePool.angleServo.setPosition(ControlConst.angleContNear);
                flow.setPower(0.4);
                brush.setPower(0.3);


                robot.shooter.setTarget(ControlConst.nearVel);
                if (time.seconds() > 0.1) {
                    robot.devicePool.wall.setPosition(0.65);
                    flow.setPower(1);
                }
                if (time.seconds() > 1.5) {
                    setState(FSM_STATES.EAT);
                }
                break;
            case DRIVE:
                break;
            case REVERSE_BRUSHES:
                targetState = FSM_STATES.REVERSE_BRUSHES;
                robot.devicePool.wall.setPosition(1);
                robot.devicePool.angleServo.setPosition(0.5);
                robot.servoMovement.anglePosNear();
                flow.setPower(0);
                brush.setPower(-0.5);

                robot.shooter.setTarget(ControlConst.nearVel);
                break;
            case REVERSE_ALL:
                targetState = FSM_STATES.REVERSE_BRUSHES;
                robot.devicePool.wall.setPosition(1);
                robot.servoMovement.anglePosNear();
                flow.setPower(-1);
                brush.setPower(-1);

                robot.shooter.setTarget(ControlConst.nearVel);
                break;
        }
    }


    @Override
    public void update() {
        if (state == targetState)
            time.reset();
        updateStates();

        FtcDashboard.getInstance().getTelemetry().addData("timer", time.seconds());
        FtcDashboard.getInstance().getTelemetry().update();

    }
}
