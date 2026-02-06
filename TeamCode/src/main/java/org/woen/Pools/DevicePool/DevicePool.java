package org.woen.Pools.DevicePool;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.woen.Utility.LEDLine;

public class DevicePool {

    public DcMotorEx odometer;

    public DcMotorEx rMF;

    public DcMotorEx lMF;

    public DcMotorEx rMB;

    public DcMotorEx lMB;


    public IMU gyro;

    public DcMotorEx shooterMotor;


    public DcMotorEx brush;

    public Servo angleServo;
    public Servo wall;
    public Servo towerAngleServo;
    public DcMotorEx flowMotor;

    public GoBildaPinpointDriver pinpoint;

    public LEDLine light1;
    public LEDLine light2;
    public LEDLine light3;
    public LEDLine light4;
    public LEDLine light5;
    public LEDLine light6;


    public void init(HardwareMap hardwareMap) {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "gun_motor_left");

        lMF = hardwareMap.get(DcMotorEx.class, "left_front_vehicle_motor");

        rMF = hardwareMap.get(DcMotorEx.class, "right_front_vehicle_motor");

        lMB = hardwareMap.get(DcMotorEx.class, "left_back_vehicle_motor");

        rMB = hardwareMap.get(DcMotorEx.class, "right_back_vehicle_motor");

        flowMotor = hardwareMap.get(DcMotorEx.class, "motor_flow");

        brush = hardwareMap.get(DcMotorEx.class, "motor_brush");

        wall = hardwareMap.get(Servo.class, "servo_door");

        angleServo = hardwareMap.get(Servo.class, "servo_angle_gun");

        towerAngleServo = hardwareMap.get(Servo.class, "servo_turn_tower");

        light1 = new LEDLine(hardwareMap, "light0", LEDLine.SignalPin.MINUS);
        light2 = new LEDLine(hardwareMap, "light1", LEDLine.SignalPin.MINUS);
        light3 = new LEDLine(hardwareMap, "light2", LEDLine.SignalPin.MINUS);
        light4 = new LEDLine(hardwareMap, "light3", LEDLine.SignalPin.MINUS);
        light5 = new LEDLine(hardwareMap, "light4", LEDLine.SignalPin.MINUS);
        light6 = new LEDLine(hardwareMap, "light5", LEDLine.SignalPin.MINUS);

    }
}
