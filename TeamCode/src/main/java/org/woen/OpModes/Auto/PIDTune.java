package org.woen.OpModes.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.woen.Modules.IntakeAndShooter.ControlConst;

import org.woen.Robot.Robot;


@Autonomous
public class PIDTune extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap);

        robot.controlAngleTower.setPosition(ControlConst.startTowPosAss);

        waitForStart();

        robot.init();

        robot.odometry.setPosition(0,0,0);




        while (opModeIsActive()){
            robot.driveTrain.fieldMovement(100,0,0);

            sleep(1000);

            robot.driveTrain.fieldMovement(0,0,0);

            sleep(1000);

        }
    }

}
