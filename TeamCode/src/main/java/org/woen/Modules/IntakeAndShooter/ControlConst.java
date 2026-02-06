package org.woen.Modules.IntakeAndShooter;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class ControlConst {
    public static double nearVel = 480;
    public static double farVel = 1600;
    public static double angleContNear = 0.5;
    public static double angleContFar = 0.85;
    public static double startTowPos = 0.495;

    public static double powerToBrushes = 1;

    public static double openWall = 1;
    public static double closeWall = 0.5;

    public static double powerToFlow = 1;

    public static Pose2D redGoal = new Pose2D(DistanceUnit.CM, -211,23, AngleUnit.DEGREES, 143);

    public static Pose2D blueGoal = new Pose2D(DistanceUnit.CM, -180,180, AngleUnit.DEGREES, 0);

}
