package org.woen.OpModes.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.woen.Modules.IntakeAndShooter.FSM_STATES;
import org.woen.Robot.Robot;


@Autonomous
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap);

        waitForStart();

        robot.init();


        robot.odometry.setPosition(-120,-40,0);


        while (opModeIsActive()){
           robot.driveTrain.fieldMovement(-100,-40,0);

            ElapsedTime timer = new ElapsedTime();

            while(timer.seconds() < 1){
                robot.intakeStateMachine.setState(FSM_STATES.SHOOT_NEAR);
                robot.update();
            }



            robot.driveTrain.fieldMovement(-120,-40,90);
            robot.driveTrain.fieldMovement(-120,-80,90);
        }
    }

}
