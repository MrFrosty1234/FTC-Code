package org.woen.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.woen.Robot.Robot;

public abstract class BaseOpMode extends LinearOpMode {
    protected final Robot robot = new Robot(this);

    public void startLoop(){

    }

    public abstract void main();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        waitForStart();
        main();
    }

}
