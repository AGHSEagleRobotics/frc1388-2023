// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  private static SendableChooser<Objective> m_autoObjective = new SendableChooser<>(); 
  
  public final class DriveTrainConstants{
    public static final int CANID_LEFT_FRONT    = 1;
    public static final int CANID_LEFT_BACK     = 2;
    public static final int CANID_RIGHT_FRONT   = 3;
    public static final int CANID_RIGHT_BACK    = 4;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public final class AutoConstants
  {
    public static final double TURN_P_VALUE = 0.03;
    public static final double TURN_P_TOLERANCE = 1.25;
    public static final double MOVE_P_VALUE = 0.04;
    public static final double MOVE_P_TOLERANCE = 0.5;
    //TODO CHANGE ALL PID & F VALUES, REVIEW
    public static final double MOVE_F_VALUE = 0;

    public static final double AUTO_TIME = 15.0; //seconds
        
    public static final double AUTO_DRIVE_SPEED = 0.5;
    public static final double AUTO_TURN_SPEED = 0.25;
    public static final double AUTO_TURN_ANGLE_MAX = 75;
  }
  
  public enum Objective
  {
    LEAVECOMMUNITY ("Leave community" ),
    SCORE( "Score" ),
    SCOREANDLEAVE ( "Score and leave community" ),
    SCORELEAVEPICKUP ( "Score, leave, pickup" ),
    CHARGESTATION ( "Balance on charge station" ),
    OVERCHARGESTATION( "Straight over charge station" ),
    CHARGESTATIONBACK( "Over station and back" );

    
    private String m_dashboardDescript; //This is what will show on dashboard
    private Objective ( String dashboardDescript )
    {
      m_dashboardDescript = dashboardDescript;
    }

  public String getDashboardDescript(){
      return m_dashboardDescript;
  }

  }
  
  public Objective getObjective()
  {
     return m_autoObjective.getSelected();
    }
  
  public final class FieldConstants
  {
    //all measurements in inches
    public static final double ROBOT_WIDTH = 28;
    public static final double ROBOT_LENGTH = 29;
    public static final double BUMPER_SIZE = 3.0;

    public static final double CHARGE_STATION_WIDTH = 76.1;

    public static final double SCORE_ZONE_TO_CHARGE_STATION = 60.7;
    public static final double SCORE_ZONE_TO_FAR_COMMUNITY = SCORE_ZONE_TO_CHARGE_STATION + CHARGE_STATION_WIDTH; //138.6 //blue right, red left
    public static final double SCORE_ZONE_TO_NEAR_COMMUNITY = 88; // blue left, red right
    public static final double SCORE_ZONE_TO_GAME_PIECE = 224;

    public static final double DOUBLE_SUBSTATION_HEIGHT = 37;
  }

}
