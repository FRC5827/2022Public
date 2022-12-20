// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.lib.util;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;


public class TrajectoryLoader {

    public static PathPlannerTrajectory _1_TopBaseCollect;
    public static PathPlannerTrajectory _1_TopBaseCollectPart2;
    public static PathPlannerTrajectory _1_ShootTaxi;
    public static PathPlannerTrajectory _1_TwoBallTaxi;
    public static PathPlannerTrajectory _2_ShootAndTaxi;
    public static PathPlannerTrajectory _3_TwoBallTaxi;
    public static PathPlannerTrajectory _4_BotBaseCollect;
    public static PathPlannerTrajectory _4_BotBaseCollectPart2;
    public static PathPlannerTrajectory _4_TwoBallTaxi;
    public static PathPlannerTrajectory _4_FiveBallPart1;
    public static PathPlannerTrajectory _4_FiveBallPart2;
    public static PathPlannerTrajectory _4_FiveBallPart3;
    public static PathPlannerTrajectory _4_FiveBallPart4;


    public enum pickTrajectory {
        _1_TopBaseCollect, _1_ShootTaxi, _1_TwoBallTaxi, _2_ShootAndTaxi, 
        _3_TwoBallTaxi, _4_BotBaseCollect, _4_TwoBallTaxi, _4_FiveBall 
    }

    public static void loadTrajectories() {
        // loads PathPlanner paths and generates trajectories
        
        _1_TopBaseCollect = PathPlanner.loadPath("1_TopBaseCollect", 3.0, 1.8);
        _1_TopBaseCollectPart2 = PathPlanner.loadPath("1_TopBaseCollectPart2", 3.0, 1.8);

        _1_ShootTaxi = PathPlanner.loadPath("1_ShootTaxi", 3.0, 1.8);
        _1_TwoBallTaxi = PathPlanner.loadPath("1_TwoBallTaxi", 3.0, 1.8);
        _2_ShootAndTaxi = PathPlanner.loadPath("2_ShootAndTaxi", 4.0, 3.0);
        _3_TwoBallTaxi = PathPlanner.loadPath("3_TwoBallTaxi", 3.0, 1.8);

        _4_BotBaseCollect = PathPlanner.loadPath("4_BotBaseCollect", 3.0, 1.8);
        _4_BotBaseCollectPart2 = PathPlanner.loadPath("4_BotBaseCollectPart2", 3.0, 1.8);
        
        _4_TwoBallTaxi = PathPlanner.loadPath("4_TwoBallTaxi", 3.0, 1.8);

        _4_FiveBallPart1 = PathPlanner.loadPath("4_FiveBallPart1", 4.0, 4.0);
        _4_FiveBallPart2 = PathPlanner.loadPath("4_FiveBallPart2", 4.0, 4.0);
        _4_FiveBallPart3 = PathPlanner.loadPath("4_FiveBallPart3", 4.0, 4.0);
        _4_FiveBallPart4 = PathPlanner.loadPath("4_FiveBallPart4", 4.0, 4.0);

        // adding paths together can be done via path1.concatenate(path2);

    }
}