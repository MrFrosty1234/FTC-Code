package org.woen.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    DcMotor rFM;
    DcMotor lFM;
    DcMotor rRM;
    DcMotor lRM;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        rFM = hardwareMap.get(DcMotor.class, "rFM");
        lFM = hardwareMap.get(DcMotor.class, "lFM");
        rRM = hardwareMap.get(DcMotor.class, "rRM");
        lRM = hardwareMap.get(DcMotor.class, "lRM");

        lFM.setDirection(DcMotorSimple.Direction.FORWARD);
        lRM.setDirection(DcMotorSimple.Direction.FORWARD);
        rFM.setDirection(DcMotorSimple.Direction.REVERSE);
        rRM.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()){
            double x = gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double angle = gamepad1.right_stick_x;


            double rM = x + y + angle;
            double rBM = x - y + angle;
            double lM = x - y - angle;
            double lBM = x + y - angle;

            rFM.setPower(rM);
            lFM.setPower(lM);
            rRM.setPower(rBM);
            lRM.setPower(lBM);
        }
    }
}
