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




    public void init(HardwareMap hardwareMap) {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "flywheel");

        lMF = hardwareMap.get(DcMotorEx.class, "br");

        rMF = hardwareMap.get(DcMotorEx.class, "bl");

        lMB = hardwareMap.get(DcMotorEx.class, "fr");

        rMB = hardwareMap.get(DcMotorEx.class, "fl");

        flowMotor = hardwareMap.get(DcMotorEx.class, "flow");

        brush = hardwareMap.get(DcMotorEx.class, "intake");

        wall = hardwareMap.get(Servo.class, "door");

        angleServo = hardwareMap.get(Servo.class, "turret");

        towerAngleServo = hardwareMap.get(Servo.class, "pitch");


    }
}
