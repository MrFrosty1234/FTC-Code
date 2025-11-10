package org.woen.Modules.Battery;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RollingAverage;

import org.woen.Modules.Interface.RobotModule;
import org.woen.Robot.Robot;
import org.woen.Utility.TimedSensorQuery;

public class Battery implements RobotModule {

    Robot robot;


    private double k = 1;

    private VoltageSensor voltageSensor = null;

    private double batVolt = 13.0;

    private final RollingAverage avgBatVolt =  new RollingAverage(8);



    public Battery(Robot robot){
        this.robot = robot;

    }

    //TODO VOLTAGE CONTROLLER




    @Override
    public void init() {
        avgBatVolt.addNumber((int) batVolt);
    }
}
