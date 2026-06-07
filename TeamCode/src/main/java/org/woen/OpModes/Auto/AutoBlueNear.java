package org.woen.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.woen.Modules.IntakeAndShooter.ControlConst;
import org.woen.Modules.IntakeAndShooter.FSM_STATES;
import org.woen.Robot.Robot;


@Autonomous
public class AutoBlueNear extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap);


        robot.init();


        robot.odometry.setPosition(-156, -100, 0);

        while (opModeInInit())
            robot.devicePool.towerAngleServo.setPosition(0.675);

        waitForStart();


        robot.driveTrain.fieldMovement(-60, -60, 0);

        ElapsedTime timer = new ElapsedTime();

        ControlConst.nearVel = 860;


        while (timer.seconds() < 1.5) {
            robot.intakeStateMachine.setState(FSM_STATES.SHOOT_NEAR);
            robot.update();
        }


        ControlConst.nearVel = 950;

        robot.devicePool.towerAngleServo.setPosition(0.85);

        robot.driveTrain.fieldMovement(32,-40,-90);
        robot.driveTrain.fieldMovement(32,-160,-90);

        robot.driveTrain.fieldMovement(32,-113,-90);
        robot.driveTrain.fieldMovement(10,-140,-90);

        robot.driveTrain.fieldMovement(32,-55,-90);

        robot.driveTrain.fieldMovement(-27,-45,-90);



        timer.reset();

        while(timer.seconds() < 1.5){
            robot.intakeStateMachine.setState(FSM_STATES.SHOOT_NEAR);
            robot.update();
        }
    }
}

