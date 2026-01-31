package org.woen.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.woen.OpModes.BaseOpMode;

public abstract class BaseAuto extends BaseOpMode {

    public final void execute(Runnable[] runnables) {
        execute(runnables, 300);
    }

    public final void execute(Runnable[] runnables, double timeoutSeconds) {
        for (Runnable action : runnables) {
            ElapsedTime elapsedTime = new ElapsedTime();
            if (opModeIsActive()) action.run();
            do robot.update();
            while (!robot.allActionsAreCompleted() && opModeIsActive() &&
                    elapsedTime.seconds() < timeoutSeconds);
        }

    }
}
