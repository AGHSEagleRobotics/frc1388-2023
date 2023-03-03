// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public final static class RumbleConstants {
    public static final double RUMBLE_PUSLE_TIME = 0.2;
    public static final double ANTI_RUMBLE_TIME = 0.2;
    public static final double RUMBLE_STRENGTH = 1.0;
    public static final int RUMBLE_OFF = 0;
    public static final int NUMBER_OF_PULSES = 2;

    public static enum RumbleSide{
        LEFT(RumbleType.kLeftRumble), 
        RIGHT(RumbleType.kRightRumble), 
        BOTH(RumbleType.kLeftRumble, RumbleType.kRightRumble), 
        NONE;
        private final RumbleType[] rumbleTypes;

        RumbleSide( RumbleType... types ){
            rumbleTypes = types;
        }
        public RumbleType[] getRumbleType(){
            return rumbleTypes;
        }
    }
  }
  
  public final class DriveTrainConstants {
    public static final int CANID_LEFT_FRONT    = 1;
    public static final int CANID_LEFT_BACK     = 2;
    public static final int CANID_RIGHT_FRONT   = 3;
    public static final int CANID_RIGHT_BACK    = 4;
    
    //Encoder stuff
    private final static double COUNTS_PER_REV = 2048;
    private final static double REVS_PER_COUNT = 1/COUNTS_PER_REV;
    private final static double WHEEL_DIAMETER_INCHES = 5.6; // make range value? //TODO when we use new robot change to 6
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

  //XBOX CONTROLLERS
  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OP_CONTROLLER_PORT = 1;

  }
 
 
  public final class AutoBalanceConstants {
    public static final double BALANCING_SPEED = 15.0;
    //public static final double LOW_SPEED = 15.0; //possibly needed for autobalancing

    public static final double BALANCED_ANGLE = 2.5;
    
    public static final double GO_UNTIL_ANGLE_SPEED = 36.0;

    public static final double GO_TO_BALANCE = 13.0;
    public static final double CHARGE_STATION_DETECTION_ANGLE = 12.0;

    public static final double DRIVE_ON_RAMP_DISTANCE = 24.0; // in inches
    public static final double DRIVE_ON_RAMP_SPEED = 24.0;
    
    public static final double NOT_BALANCED_TICKS = 10.0; // 20 ticks per second
  }
    
	public enum Objective
  {
    SITSTILL("SitStillLookPretty"),
    LEAVECOMMUNITY( "Leave community" ),
    SCORE( "Score" ),
    SCOREANDLEAVE ( "Score and leave community" ),
    SCORELEAVEPICKUP ( "Score, leave, pickup" ),
    CHARGESTATION ( "Balance on charge station" ),
    SCORETHENCHARGE( "Score then charge balance" ),
    OVERCHARGESTATION( "Straight over charge station" ),
    CHARGESTATIONBACK( "Over station and back" ),
    SCOREOVERCHARGEBACK( "Score, Over, Back" );

    
    private String m_dashboardDescript; //This is what will show on dashboard
    private Objective ( String dashboardDescript ) {
      	m_dashboardDescript = dashboardDescript;
  }

    public String getDashboardDescript() {
      return m_dashboardDescript;
    }
  }
  
  public static class AutoConstants {
    //TEST P VALUE LATER
    
    public static final double TURN_P_VALUE = 0.0125;
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



      public enum Objective {
        SITSTILL ( "LookPretty"),
        LEAVECOMMUNITY( "LeaveCommunity" ),
        SCORE( "ArmScore" ),
        SCOREANDLEAVE ( "Score, Leave" ),
        SCORELEAVEPICKUP ( "Score, Leave, Pickup" ),
        CHARGESTATION ( "Balance" ),
        SCORETHENCHARGE( "Score, Balance" ),
        OVERCHARGESTATION( "OverChargeStation" ),
        CHARGESTATIONBACK( "OverCharge, Return" );
    
        public static final Objective Default = SITSTILL;
      
        private String m_dashboardDescript; //This is what will show on dashboard
        private Objective ( String dashboardDescript ) {
          m_dashboardDescript = dashboardDescript;
        }
    
        public String getDashboardDescript() {
          return m_dashboardDescript;
        }  
      }

      public enum Position {
        NEAR("FAR (bRight rLeft)"),
        MID("MID"),
        FAR("NEAR (bLeft rRight )");
    
        public static final Position Default = NEAR;
    
        private String m_dashboardDescript; //This is what will show on dashboard
        private Position ( String dashboardDescript ) {
          m_dashboardDescript = dashboardDescript;
        }
    
        public String getDashboardDescript() {
          return m_dashboardDescript;
        }
    
        public static final double AUTO_POSITION_4_DISTANCE_TO_WALL_BALL = 42;
        public static final double AUTO_POSITION_4_DISTANCE_TAXI = 7;
        public static final double AUTO_POSITION_4_DISTANCE_2_BALL_BACK = -37; //was -28
        public static final double AUTO_POSITION_4_DISTANCE_3_BALL = -26;
      }
    }


  public final class GrabberConstants {
    public static final int GRABBER_CANID = 7;
    public static final int GRABBER_LIMIT_SWITCH_ID = 0;
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
    public static final double ROBOT_WIDTH = 28;
    public static final double ROBOT_LENGTH = 29; 
    public static final double BUMPER_SIZE = 3.0; 
    public static final double ROBOT_WIDTH_TOTAL = ROBOT_WIDTH + BUMPER_SIZE + BUMPER_SIZE;
    public static final double ROBOT_LENGTH_TOTAL = ROBOT_LENGTH + BUMPER_SIZE + BUMPER_SIZE; //even with the u-shape of the robot, side bumpers extend to the front and add 3 inches

    public static final double CHARGE_STATION_LENGTH = 76.1;

    public static final double SCORE_ZONE_TO_CHARGE_STATION = 60.7;
    public static final double SCORE_ZONE_TO_FAR_COMMUNITY = SCORE_ZONE_TO_CHARGE_STATION + CHARGE_STATION_LENGTH; //138.6 //blue right, red left
    public static final double SCORE_ZONE_TO_NEAR_COMMUNITY = 88; // blue left, red right
    public static final double SCORE_ZONE_TO_GAME_PIECE = 224;

    public static final double DOUBLE_SUBSTATION_HEIGHT = 37;
  }

  public final class Logging {
    public static final boolean LOG_ACCELERATION = true;
  }

}
