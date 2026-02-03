package org.woen.OpModes;


import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Mat;
import org.woen.Modules.IntakeAndShooter.ControlConst;
import org.woen.Modules.IntakeAndShooter.FSM_STATES;
import org.woen.Robot.Robot;
import org.woen.Utility.PID.PID;
import org.woen.Utility.Vector2D;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TeleOp extends LinearOpMode {

    DcMotor rFM;
    DcMotor lFM;
    DcMotor rRM;
    DcMotor lRM;

    Robot robot;

    PID angleController = new PID(0.005,0,0.0001,0,0,0,0);

    public static double aimXY = 64 * 2.54;




    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(this, hardwareMap);

        waitForStart();
        lFM = hardwareMap.get(DcMotorEx.class, "left_front_vehicle_motor");

        rFM = hardwareMap.get(DcMotorEx.class, "right_front_vehicle_motor");

        lRM = hardwareMap.get(DcMotorEx.class, "left_back_vehicle_motor");

        rRM = hardwareMap.get(DcMotorEx.class, "right_back_vehicle_motor");
        lFM.setDirection(DcMotorSimple.Direction.FORWARD);
        lRM.setDirection(DcMotorSimple.Direction.FORWARD);
        rFM.setDirection(DcMotorSimple.Direction.REVERSE);
        rRM.setDirection(DcMotorSimple.Direction.REVERSE);

        lFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

         robot.init();
        while (opModeInInit())
            robot.odometry.setPosition(ControlConst.redGoal.getX(CM),ControlConst.redGoal.getY(CM), ControlConst.redGoal.getHeading(AngleUnit.DEGREES));

        Vector2D aimVec = new Vector2D(-aimXY, aimXY).getSubtracted(new Vector2D(robot.odometry.getPosX(), robot.odometry.getPosY()));

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_y;
            double y = gamepad1.left_stick_x;
            double angle = -gamepad1.right_stick_x;


            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


            if (gamepad1.right_bumper) {
                robot.intakeStateMachine.setState(FSM_STATES.SHOOT_NEAR);
            }

            if(gamepad1.left_trigger > 0.1){
                robot.intakeStateMachine.setState(FSM_STATES.REVERSE_BRUSHES);
            }
            if(gamepad1.left_trigger < 0.1 && robot.intakeStateMachine.state != FSM_STATES.SHOOT_NEAR){
                robot.intakeStateMachine.setState(FSM_STATES.EAT);
            }

            if(gamepad1.left_bumper && robot.intakeStateMachine.state != FSM_STATES.SHOOT_NEAR){
                robot.intakeStateMachine.setState(FSM_STATES.REVERSE_ALL);
            }

            telemetry.addData("x", robot.odometry.getPosX());

            telemetry.addData("y", robot.odometry.getPosY());

            telemetry.addData("h", robot.odometry.getPosH());

            telemetry.update();

            if(gamepad1.right_trigger > 0.1){
                angle = angleController.update(minAngleError(Math.toRadians(aimVec.getAngle()) - backAngle(robot.odometry.getPosH())));
            }
            else{
                angleController.reset();
            }

            robot.driveTrain.setPower(x, y, angle);

             robot.update();
        }
    }

    double backAngle(double angle) {
        return angle + (angle > 0 ? -180 : 180);
    }

    double minAngleError(double rawAngleError) {
        return rawAngleError + (rawAngleError < -180 ? 360 : rawAngleError > 180 ? -360 : 0);
    }

}
