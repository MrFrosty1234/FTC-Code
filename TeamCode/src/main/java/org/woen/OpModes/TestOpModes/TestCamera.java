package org.woen.OpModes.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.FtcRobotControllerServiceState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.woen.Modules.Camera.Camera;
import org.woen.Modules.Camera.NewCamera;
import org.woen.Modules.Interface.RobotModule;
import org.woen.Robot.Robot;


@TeleOp
public class TestCamera extends LinearOpMode {


    @Override
    public void runOpMode(){

        NewCamera camera = new NewCamera(hardwareMap);

        camera.init();

        waitForStart();

        while (opModeIsActive()){
            int id = camera.getId();

            FtcDashboard.getInstance().getTelemetry().addData("id", id);

            FtcDashboard.getInstance().getTelemetry().update();

            camera.update();
        }

    }


}
