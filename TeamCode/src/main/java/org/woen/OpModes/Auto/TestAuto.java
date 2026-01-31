package org.woen.OpModes.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.woen.Modules.DriveTrain.DriveTrainMode;

@Autonomous
public class TestAuto extends BaseAuto{

    @Override
    public void startLoop() {
        robot.driveTrain.setMode(DriveTrainMode.AUTO);
    }

    Runnable[] goForw = {
            ()->{
                robot.driveTrain.setFieldPos(100,0,0);
            },
            () -> robot.driveTrain.setFieldPos(0,0,0)
    };

    Runnable[] goBack = {
            ()->{
                robot.driveTrain.setFieldPos(0,0,0);
            }
    };


    @Override
    public void main() {
        execute(goForw, 10);

    }
}
