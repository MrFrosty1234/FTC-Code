package org.woen.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous
public class UneversalAuto extends LinearOpMode {



    @Override
    public void runOpMode() {
        STATES states = STATES.KIWI;

        DcMotor lF = hardwareMap.get(DcMotor.class, "");
        DcMotor rF = hardwareMap.get(DcMotor.class, "");
        DcMotor lB = hardwareMap.get(DcMotor.class, "");
        DcMotor rB = hardwareMap.get(DcMotor.class, "");
        while (opModeInInit()) {
            if (gamepad1.dpad_left) {
                states = STATES.TANK;
            }
            if (gamepad1.dpad_up) {
                states = STATES.KIWI;
            }
            if (gamepad1.dpad_right) {
                states = STATES.MEC;
            }
        }

        while (opModeIsActive()) {
            switch (states) {
                case MEC:
                    rB.setPower(1);
                    rF.setPower(1);
                    lF.setPower(1);
                    lB.setPower(1);
                case TANK:
                    rB.setPower(1);
                    rF.setPower(1);
                    lF.setPower(1);
                    lB.setPower(1);
            }
        }
    }

}

enum STATES {
    KIWI, TANK, MEC
}
