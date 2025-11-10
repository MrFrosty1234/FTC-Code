package org.woen.Pools.PositionPool;

import com.acmerobotics.dashboard.config.Config;

import org.woen.Math.Pose2D;

@Config
public class Positions {
    public static Pose2D blueGoal =
            new Pose2D(-183 , 183, 45);

    public static Pose2D redGoal =
            new Pose2D(183 , 183, -45);


}
