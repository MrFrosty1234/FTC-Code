package org.woen.Pools.DevicePool;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

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

    }
}
