package org.woen.Pools.DevicePool;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.woen.Modules.Camera.Camera;

import java.util.concurrent.CancellationException;

public class DevicePool {

    public DcMotorEx sideOdometer;
    public DcMotorEx leftOdometer;
    public  DcMotorEx rightOdometer;

    public DcMotorEx rM;

    public DcMotorEx lM;

    public IMU gyro;

    public DcMotorEx rShooter;

    public DcMotorEx lShooter;


    public WebcamName camera;

    private HardwareMap hardwareMap;

    public Servo lEatServo;

    public Servo cEatServo;

    public Servo rEatServo;

    public Servo rAngleServo;

    public Servo cAngleServo;

    public Servo lAngleServo;

    public Servo wall;



    public DevicePool (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public void init(){
        sideOdometer = hardwareMap.get(DcMotorEx.class, "sideOdometer");

        leftOdometer = hardwareMap.get(DcMotorEx.class, "leftOdometer");

        rightOdometer = hardwareMap.get(DcMotorEx.class, "rightOdometer");

        gyro = hardwareMap.get(IMU.class, "gyro");

        rM = hardwareMap.get(DcMotorEx.class, "motorR");

        lM = hardwareMap.get(DcMotorEx.class, "motorL");


        camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        rShooter = hardwareMap.get(DcMotorEx.class, "gunR");

        lShooter = hardwareMap.get(DcMotorEx.class, "gunL");

        lEatServo = hardwareMap.get(Servo.class, "shotL");

        rEatServo = hardwareMap.get(Servo.class, "shotR");

        cEatServo = hardwareMap.get(Servo.class, "shotC");

        wall = hardwareMap.get(Servo.class, "wall");

        rAngleServo = hardwareMap.get(Servo.class, "aimL");

        lAngleServo = hardwareMap.get(Servo.class, "aimR");

        cAngleServo = hardwareMap.get(Servo.class, "aimC");

    }
}
