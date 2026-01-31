package org.woen.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.woen.Modules.Camera.Camera;
import org.woen.Modules.DriveTrain.DriveTrain;
import org.woen.Modules.IntakeAndShooter.FSM.IntakeStateMachine;
import org.woen.Modules.IntakeAndShooter.Shooter.Shooter;
import org.woen.Modules.Odometery.Odometry;
import org.woen.Modules.Interface.RobotModule;
import org.woen.Modules.Servo.ServoMovement;
import org.woen.Pools.DevicePool.DevicePool;

import java.util.Arrays;
import java.util.List;

public class Robot {


    public LinearOpMode linearOpMode;
    public ElapsedTime timer;

    public Robot(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
    }


    public DevicePool devicePool = new DevicePool(linearOpMode.hardwareMap);
    public Odometry odometry = new Odometry(this);
    public DriveTrain driveTrain = new DriveTrain(this);
    public Camera camera = new Camera(this);
    public ServoMovement servoMovement = new ServoMovement(this);
    public IntakeStateMachine intakeStateMachine = new IntakeStateMachine(this);

    public Shooter shooter = new Shooter(this);

    List<LynxModule> allHubs = linearOpMode.hardwareMap.getAll(LynxModule.class);

    private final RobotModule[] robotModule = new RobotModule[]{
            odometry, driveTrain
    };

    public void init(){

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);//if we watch electronics <= 1 switch to auto
        }

        for (RobotModule robotModule1 : robotModule)
            robotModule1.init();

    }

    public void updateRevBulkCache() {
        for (LynxModule module : allHubs)
            module.clearBulkCache();
    }

    public void update(){
        updateRevBulkCache();
        for(RobotModule robotModule1 : robotModule){
            robotModule1.update();
        }
    }
    public boolean allActionsAreCompleted() {
        return Arrays.stream(robotModule).allMatch(RobotModule::isAtTarget);
    }



}
