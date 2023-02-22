// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public final class DriveTrainConstants{
    public static final int CANID_LEFT_FRONT    = 1;
    public static final int CANID_LEFT_BACK     = 2;
    public static final int CANID_RIGHT_FRONT   = 3;
    public static final int CANID_RIGHT_BACK    = 4;
    
    //Encoder stuff
    private final static double COUNTS_PER_REV = 2048;
    private final static double REVS_PER_COUNT = 1/COUNTS_PER_REV;
    private final static double WHEEL_DIAMETER_INCHES = 6.0; // make range value?
    private final static double FALCON_TO_SIMPLE_BOX_GEAR_RATIO = 1/4.67;
    private final static double SIMPLE_BOX_TO_WHEELS_RATIO = 12.0/30.0; //12 sprockets simple box to 30 sprockets wheel
    public final static double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;
    public final static double INCHES_PER_ENCODER_UNITS = 
      REVS_PER_COUNT *
      FALCON_TO_SIMPLE_BOX_GEAR_RATIO *
      SIMPLE_BOX_TO_WHEELS_RATIO *
      WHEEL_CIRCUMFERENCE;
    
    public final static int SENSOR_CYCLES_PER_SECOND = 10;   // sensor velocity period is 100 ms
    public final static int SEC_PER_MIN = 60;

    //tuned for constant speed drive lint2023 
    public static final double GAINS_VELOCITY_F = 0.055;
    public static final double GAINS_VELOCITY_P = 0.03; 
    public static final double GAINS_VELOCITY_I = 0.0001;
    public static final double GAINS_VELOCITY_D = 0;
    public static final int PID_IDX = 0;
    public static final double CLOSED_LOOP_RAMP_RATE = 3;
  } 

  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OP_CONTROLLER_PORT = 1;

  }
 
 
  public final class AutoBalanceConstants
  {
    public static final double HIGH_SPEED = 15.0;
    public static final double LOW_SPEED = 15.0;

  }

  public enum Objective
  {
    LEAVECOMMUNITY( "Leave community" ),
    SCORE( "Score" ),
    SCOREANDLEAVE ( "Score and leave community" ),
    SCORELEAVEPICKUP ( "Score, leave, pickup" ),
    CHARGESTATION ( "Balance on charge station" ),
    SCORETHENCHARGE( "Score then charge balance" ),
    OVERCHARGESTATION( "Straight over charge station" ),
    CHARGESTATIONBACK( "Over station and back" );

    
    private String m_dashboardDescript; //This is what will show on dashboard
    private Objective ( String dashboardDescript ) {
      	m_dashboardDescript = dashboardDescript;
  }

    public String getDashboardDescript() {
      return m_dashboardDescript;
	    }  
  }
  public final class AutoConstants {
    //TEST P VALUE LATER
    
    public static final double TURN_P_VALUE = 0.03;
    public static final double TURN_P_TOLERANCE = 1.25;
    public static final double MOVE_P_VALUE = 0.04;
    public static final double MOVE_P_TOLERANCE = 0.5;

    public static final double MOVE_F_VALUE = 0;
 
    public static final int USB_CAMERACOLOR = 0; //FIXME Not used?

    public static final double AUTO_SHOOT_RPM = 3700; 
    public static final double SHOOTER_TIMER_1 = 1;
    public static final double SHOOTER_TIMER_2 = 1.7;

    public static final double ENCODER_DISTANCE_CUTOFF = 1.0; //TODO change - is this cutoff??
    public static final double AUTO_DRIVE_SPEED = 0.5;
}

  public enum Position {
    A("A"),
    B("B"),
    C("C");

    private String m_dashboardDescript; //This is what will show on dashboard
    private Position ( String dashboardDescript ){
      m_dashboardDescript = dashboardDescript;
    }

    public String getDashboardDescript(){
      return m_dashboardDescript;
    }

    public static final double AUTO_POSITION_4_DISTANCE_TO_WALL_BALL = 42;
    public static final double AUTO_POSITION_4_DISTANCE_TAXI = 7;
    public static final double AUTO_POSITION_4_DISTANCE_2_BALL_BACK = -37; //was -28
    public static final double AUTO_POSITION_4_DISTANCE_3_BALL = -26;
  }

  public final class GrabberConstants {
    /**when the grabber limit switch is triggered, the encoder knows it is at this value and resets, measured in motor rotations */
    public static final int GRABBER_POSITION_AT_LIMIT_SWITCH = 50;
    /** the position of the motor when the grabber is open, measured in motor rotations */
    public static final int GRABBER_POSITION_OPEN = 100;
    /** the position of the motor when the grabber is closed, measured in motor rotations */
    public static final int GRABBER_POSITION_CLOSED = 0;
    /** the dead band tolerance for the grabber, measured in motor rotations */
    public static final int GRABBER_ENCODER_DEADBAND = 5;
  }

  public final class ArmConstants {
    public static final int DEADBAND = 10;
    public static final int PRIMARY_ARM_POSITION_AT_ENCODER = 5; // TODO  definitely change this
    public static final int PRIMARY_ARM_POSITION_UP = 10; // TODO  definitely change this
    public static final int PRIMARY_ARM_POSITION_DOWN = 3; // TODO  definitely change this

    public static final int MID_ARM_POSITION_AT_ENCODER = 5; // TODO  definitely change this
    public static final int MID_ARM_POSITION_UP = 10; // TODO  definitely change this
    public static final int MID_ARM_POSITION_DOWN = 3; // TODO  definitely change this
  }

  
    
  public final class FieldConstants {
    //all measurements in inches
    public static final double ROBOT_WIDTH = 28; //delete?
    public static final double ROBOT_LENGTH = 29; //delete?
    public static final double BUMPER_SIZE = 3.0; //delete?
    public static final double ROBOT_WIDTH_BUMPERS = 31;
    public static final double ROBOT_LENGTH_BUMPERS = 32;

    public static final double CHARGE_STATION_WIDTH = 76.1;

    public static final double SCORE_ZONE_TO_CHARGE_STATION = 60.7;
    public static final double SCORE_ZONE_TO_FAR_COMMUNITY = SCORE_ZONE_TO_CHARGE_STATION + CHARGE_STATION_WIDTH; //138.6 //blue right, red left
    public static final double SCORE_ZONE_TO_NEAR_COMMUNITY = 88; // blue left, red right
    public static final double SCORE_ZONE_TO_GAME_PIECE = 224;

    public static final double DOUBLE_SUBSTATION_HEIGHT = 37;
  }

  public final class Logging {
    public static final boolean LOG_ACCELERATION = true;
  }

}
