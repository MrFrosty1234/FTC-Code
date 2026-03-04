package org.woen.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.woen.Modules.IntakeAndShooter.ControlConst;
import org.woen.Modules.IntakeAndShooter.FSM_STATES;
import org.woen.OpModes.EvilEdje.Boot;
import org.woen.Robot.Robot;


@Autonomous
public class AutoBlueNear extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap);


        robot.init();


        robot.odometry.setPosition(-132, -128,53);

        while (opModeInInit())
            robot.devicePool.towerAngleServo.setPosition(ControlConst.startTowPosAss + 0.01);

        waitForStart();



        while (opModeIsActive()){
            robot.driveTrain.fieldMovement(-60,-36,54);

            ElapsedTime timer = new ElapsedTime();



            while(timer.seconds() < 1.5){
                robot.intakeStateMachine.setState(FSM_STATES.SHOOT_NEAR);
                robot.update();
            }

            robot.driveTrain.fieldMovement(-27,-48,-95);
            robot.intakeStateMachine.setState(FSM_STATES.EAT);
            robot.devicePool.towerAngleServo.setPosition(0.75);
            robot.driveTrain.fieldMovement(-31,-120,-95);

            sleep(1000);

            robot.driveTrain.fieldMovement(-27,-45,-95);

            timer.reset();

            while(timer.seconds() < 1.5){
                robot.intakeStateMachine.setState(FSM_STATES.SHOOT_NEAR);
                robot.update();
            }


            robot.driveTrain.fieldMovement(32,-55,-90);

            robot.driveTrain.fieldMovement(32,-145,-90);
            sleep(750);

            robot.driveTrain.fieldMovement(32,-55,-90);

            robot.driveTrain.fieldMovement(-27,-45,-90);

            timer.reset();

            while(timer.seconds() < 1.5){
                robot.intakeStateMachine.setState(FSM_STATES.SHOOT_NEAR);
                robot.update();
            }
            robot.driveTrain.fieldMovement(85,-55,-90);

            robot.driveTrain.fieldMovement(85,-145,-90);

            sleep(750);

            robot.driveTrain.fieldMovement(85,-55,-90);

            robot.driveTrain.fieldMovement(-27,-45,-90);

            timer.reset();

            while(timer.seconds() < 1.5){
                robot.intakeStateMachine.setState(FSM_STATES.SHOOT_NEAR);
                robot.update();
            }
            robot.driveTrain.fieldMovement(0,-55,-90);
            break;
        }
    }

}
