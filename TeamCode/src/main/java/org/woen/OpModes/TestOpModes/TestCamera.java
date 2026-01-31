package org.woen.OpModes.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.FtcRobotControllerServiceState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.woen.Modules.Camera.Camera;
import org.woen.Modules.Camera.NewCamera;
import org.woen.Modules.Interface.RobotModule;
import org.woen.Robot.Robot;

import javax.xml.transform.stream.StreamSource;



@Config
@TeleOp
public class TestCamera extends LinearOpMode {




    public static double greenG = 0;
    public static double greenR = 0;
    public static double greenB = 0;
    public static double purpleG = 0;
    public static double purpleR = 0;
    public static double purpleB = 0;
    public static double voidG = 0;
    public static double voidR = 0;
    public static double voidB = 0;

    @Override
    public void runOpMode(){

        NewCamera camera = new NewCamera(hardwareMap);

        camera.init();

        RevColorSensorV3 color = hardwareMap.get(RevColorSensorV3.class, "leftColor");


        waitForStart();

        while (opModeIsActive()){
            int id = camera.getId();

            FtcDashboard.getInstance().getTelemetry().addData("id", id);

            NormalizedRGBA detectedColor = color.getNormalizedColors();

            FtcDashboard.getInstance().getTelemetry().addData("green",detectedColor.green * 1000);

            FtcDashboard.getInstance().getTelemetry().addData("red", detectedColor.red * 1000);

            FtcDashboard.getInstance().getTelemetry().addData("blue", detectedColor.blue * 1000);



           /* if(detectedColor.red > voidR && detectedColor.blue > voidB && detectedColor.green > voidG){
                if(detectedColor.green > greenG && detectedColor.blue > greenB &&  detectedColor.red > greenR)
                //    FtcDashboard.getInstance().getTelemetry().addData("green", true);
                if(detectedColor.green > purpleG && detectedColor.blue > purpleB &&  detectedColor.red > purpleR)
                    FtcDashboard.getInstance().getTelemetry().addData("green", true);
            }
            else{
                FtcDashboard.getInstance().getTelemetry().addData("void", true);
            }

            */


            FtcDashboard.getInstance().getTelemetry().update();

            camera.update();
        }

    }


}
