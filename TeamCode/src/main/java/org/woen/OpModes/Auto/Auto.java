package org.woen.OpModes.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.woen.Robot.Robot;


@TeleOp
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap);

        waitForStart();

        robot.init();

        robot.odometry.setPosition(0,0,0);

        while (opModeIsActive()){
            FtcDashboard.getInstance().getTelemetry().addLine("inited");
            FtcDashboard.getInstance().getTelemetry().update();
            robot.driveTrain.fieldMovement(0,0,90);
            FtcDashboard.getInstance().getTelemetry().addLine();
            FtcDashboard.getInstance().getTelemetry().addLine("1st finished");
            FtcDashboard.getInstance().getTelemetry().update();
            robot.driveTrain.fieldMovement(0,0,-90);
        }
    }
}
