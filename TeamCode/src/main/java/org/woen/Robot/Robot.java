package org.woen.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;



import org.woen.Modules.DriveTrain.DriveTrain;
import org.woen.Modules.IntakeAndShooter.FSM.IntakeStateMachine;
import org.woen.Modules.IntakeAndShooter.Shooter.Shooter;
import org.woen.Modules.Odometery.Odometry;

import org.woen.Modules.Servo.ServoMovement;
import org.woen.Pools.DevicePool.DevicePool;


public class Robot {


    public LinearOpMode linearOpMode;
    public ElapsedTime timer;

    public HardwareMap hardwareMap;

    public Robot(LinearOpMode linearOpMode, HardwareMap hardwareMap){
        this.linearOpMode = linearOpMode;
        this.hardwareMap = hardwareMap;
    }


    public DevicePool devicePool = new DevicePool();
    public Odometry odometry = new Odometry(this);
    public DriveTrain driveTrain = new DriveTrain(this);

    public ServoMovement servoMovement = new ServoMovement(this);
    public IntakeStateMachine intakeStateMachine = new IntakeStateMachine(this);

    public Shooter shooter = new Shooter(this);

    public void init(){
        devicePool.init(hardwareMap);
       odometry.init();
       driveTrain.init();
       servoMovement.init();
       intakeStateMachine.init();
       shooter.init();

    }

    public void update(){
        odometry.          update();
        driveTrain.        update();
        servoMovement.     update();
        intakeStateMachine.update();
        shooter.           update();
    }



}
