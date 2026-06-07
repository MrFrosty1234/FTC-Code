package org.woen.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.woen.Robot.Robot;

@Autonomous

public class Reset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, hardwareMap);

        waitForStart();

        robot.init();

        robot.odometry.reset();
    }
}
