package org.woen.MatchData;

import org.woen.Math.Pose2D;
import org.woen.Utility.Team.TEAM;

public class MatchData {

    public static Pose2D startPos =
            new Pose2D(-180,180,0);

    public static TEAM team = TEAM.RED;

    public static Pose2D goal =
            new Pose2D(-180,180, 45);

}
