package org.woen.OpModes;


import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TeleOp extends LinearOpMode {

    DcMotor rSides;
    DcMotor lSides;
    DcMotor rMid;
    DcMotor lMid;

    Servo claw;
    Servo arm;
    Servo rDif;
    Servo lDif;
    Servo stageLeft;
    Servo stageRight;

    public static double openClaw = 0.25;
    public static double closeClaw = 0.8;

    public static double armIn = 0.9;
    public static double armFar = 0.1;

    public static double rightUp = 0.25;
    public static double leftUp = 0.25;

    public static double rightDown = 0.;
    public static double leftDown = 0.;

    public static double lStart = 0.5;
    public static double rStart = 0.545;

    public static double k = 2;

    public static double kArm = 12;

    double gamepdaStickOld = 0;


    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();
        lSides = hardwareMap.get(DcMotorEx.class, "lS");

        rSides = hardwareMap.get(DcMotorEx.class, "rS");

        lMid = hardwareMap.get(DcMotorEx.class, "lM");

        rMid = hardwareMap.get(DcMotorEx.class, "rM");

        lDif = hardwareMap.get(Servo.class, "lDif");
        rDif = hardwareMap.get(Servo.class, "rDif");

        lSides.setDirection(DcMotorSimple.Direction.FORWARD);
        lMid.setDirection(DcMotorSimple.Direction.FORWARD);
        rSides.setDirection(DcMotorSimple.Direction.REVERSE);
        rMid.setDirection(DcMotorSimple.Direction.REVERSE);

        lSides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lMid.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rMid.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "arm");
        stageLeft = hardwareMap.get(Servo.class, "stageLeft");
        stageRight = hardwareMap.get(Servo.class, "stageRight");


        claw.setPosition(openClaw);
        arm.setPosition(armIn);
        stageRight.setPosition(rightDown);
        stageLeft.setPosition(leftDown);
        lDif.setPosition(lStart);
        rDif.setPosition(rStart);

        double rPos = rStart;
        double lPos = lStart;

        boolean oldBut = true;

        boolean firstTouch = true;

        ElapsedTime timer = new ElapsedTime();

        ElapsedTime armTimer = new ElapsedTime();

        double armPos = armIn;
        double armPosOld = armIn;

        armTimer.reset();

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_y;
            double angle = -gamepad1.left_stick_x;


            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            lSides.setPower(x - angle);
            rSides.setPower(x + angle);
            lMid.setPower(x - angle);
            rMid.setPower(x + angle);

            if (gamepad1.right_bumper) {

                if (firstTouch) {
                    timer.reset();
                    firstTouch = false;
                }

                rPos = rPos - timer.seconds() / k;
                lPos = lPos - timer.seconds() / k;
                lDif.setPosition(Range.clip(lPos, 0, 1));
                rDif.setPosition(Range.clip(rPos, 0, 1));
            }

            if (gamepad1.left_bumper) {

                if (firstTouch) {
                    timer.reset();
                    firstTouch = false;
                }

                rPos = rPos + timer.seconds() / k;
                lPos = lPos + timer.seconds() / k;
                lDif.setPosition(Range.clip(lPos, 0, 1));
                rDif.setPosition(Range.clip(rPos, 0, 1));
            }

            if (!gamepad1.left_bumper && !gamepad1.right_bumper)
                firstTouch = true;

            if(abs(gamepad1.right_stick_y) != 0) {
                armPos = armPosOld + armTimer.seconds() * (gamepad1.right_stick_y) ;

                if(armPos > 0.95)
                    armPos = 0.95;
                if(armPos < 0.5)
                    armPos = 0.5;
                armPosOld = armPos;

            }else{
                 armPos = armPosOld;
                armTimer.reset();
            }


            gamepdaStickOld = gamepad1.right_stick_y;
            telemetry.addData("armPos", armPos);
            telemetry.addData("armTimer", armTimer);
            telemetry.update();
            arm.setPosition(armPos);


        }
    }

}
