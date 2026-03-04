package org.woen.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.woen.Modules.IntakeAndShooter.ControlConst;
import org.woen.Modules.IntakeAndShooter.FSM_STATES;
import org.woen.OpModes.EvilEdje.Boot;
import org.woen.Robot.Robot;


@Autonomous
public class AutoBlueFar extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap);


        robot.init();


        robot.odometry.setPosition(143, -9,0);

        while (opModeInInit())
            robot.devicePool.towerAngleServo.setPosition(ControlConst.startTowPosAss + 0.01);

        waitForStart();



        while (opModeIsActive()){
            robot.driveTrain.fieldMovement(116,-100,0);

            break;
        }
    }

}
