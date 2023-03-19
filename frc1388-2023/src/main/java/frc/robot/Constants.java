// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.security.PublicKey;
import java.util.zip.Inflater;
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
    private final static double WHEEL_DIAMETER_INCHES = 6; // make range value? //TODO when we use new robot change to 6
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

    public static final double DRIVE_SCALING_CONSTANT = 5;
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
  
  public static class AutoConstants {
    //TEST P VALUE LATER
    public static final double TURN_P_VALUE = 0.0125;
    public static final double TURN_P_TOLERANCE = 1.25;
    public static final double MOVE_P_VALUE = 0.045;
    public static final double MOVE_P_TOLERANCE = 0.5;

    public static final double MOVE_MIN_SPEED = 0.15;

    public static final double CURVE_P_VALUE = 0.025;
    public static final double CURVE_MAX = 0.25;

    public static final double TURN_MIN_SPEED_STOPPED = 0.12;
    public static final double TURN_MIN_SPEED_MOVING = 0.075;
    public static final double TURN_MIN_SPEED_THRESHOLD = 2;
    
    public static final double MOVE_F_VALUE = 0;
 
    public static final int USB_CAMERACOLOR = 0; //FIXME Not used?

    public static final double AUTO_SHOOT_RPM = 3700; 
    public static final double SHOOTER_TIMER_1 = 1;
    public static final double SHOOTER_TIMER_2 = 1.7;

    public static final double ENCODER_DISTANCE_CUTOFF = 1.0; //TODO change - is this cutoff??
    public static final double AUTO_DRIVE_SPEED = 0.5;

    public static double AUTO_TURN_ANGLE = 180;

    //we need extra distance because the encoder takes distance regardless of angle. 
    //This makes going up the charge station come up a bit short. 
    public static double CHARGE_STATION_MAGIC_NUM = 25;

      public enum Objective {
        SITSTILL ( "LookPretty"),
        LEAVECOMMUNITY( "LeaveCommunity" ),
        SCORE( "ArmScore" ),
        SCOREANDLEAVE ( "Score, Leave" ),
        SCORELEAVEPICKUP ( "Score, Leave, Pickup" ),
        CHARGESTATION ( "Balance" ),
        SCORETHENCHARGE( "Score, Balance" ),
        OVERCHARGESTATION( "OverChargeStation" ),
        CHARGESTATIONBACK( "OverCharge, Balance" ), 
        SCOREOVERCHARGEBACK( "Score, OverCharge, Balance" ),
        HYBRIDSCORE("score in hybrid node");
    
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
        SUBSTATION("SUBSTATION"),
        MID("MID"),
        FAR("FAR");
    
        public static final Position Default = SUBSTATION;
    
        private String m_dashboardDescript; //This is what will show on dashboard
        private Position ( String dashboardDescript ) {
          m_dashboardDescript = dashboardDescript;
        }
    
        public String getDashboardDescript() {
          return m_dashboardDescript;
        }
      }
  }


  public final class ArmConstants {

    // general
    /**arm rotations */
    public static final double DEADBAND = 0.04;
    /** the distance, measured in rotations, of the arm position parallel to the ground to the arm position stowed */
    public static final double FLAT_TO_UP = 0.27;
    
    // primary arm
    public static final double FALCON_TICS_PER_REV = 2048;
    public static final double PRIMARY_ARM_VERSA_RATION = 60;
    public static final double GEAR_BOX_TO_ARM_RATIO = 4.66667;
    public static final double ENCODER_UNITS_PER_PRIMARY_ARM_ROTATIONS = FALCON_TICS_PER_REV * PRIMARY_ARM_VERSA_RATION * GEAR_BOX_TO_ARM_RATIO;
    
    public static final int PRIMARY_ARM_CANID = 5;
    public static final int PRIMARY_ARM_LIMIT_SWITCH_DIO_ID = 0;

    public static final double PRIMARY_ARM_POSITION_AT_LIMIT_SWITCH = 0;
    public static final double PRIMARY_ARM_POSITION_UP = ENCODER_UNITS_PER_PRIMARY_ARM_ROTATIONS * 0.25; 
    public static final double PRIMARY_ARM_POSITION_DOWN = 0; 

    public static final double ARM_MAX_EXTEND_LOW = 0.13;
    public static final double ARM_MAX_EXTEND_HIGH = 0.25;
    public static final double PRIMARY_ARM_POSITION_MAX = 0.2;

    // wrist arm conversion factor
    public static final double WRIST_MOTOR_ROTATIONS_PER_WRIST_ARM_ROTATIONS = 2048 * 16; // TODO check this

    // wrist arm
    public static final int WRIST_CANID = 8;
    // public static final double WRIST_MOTOR_DEADBADND = 1;
    public static final int WRIST_LIMIT_SWITCH_DIO_ID = 1;

    public static final double WRIST_POSITION_AT_LIMIT_SWITCH = 0;
    public static final double WRIST_POSITION_UP = WRIST_MOTOR_ROTATIONS_PER_WRIST_ARM_ROTATIONS * 0.45;
    
    public static final double WRIST_POSITION_DOWN = 0;
    public static final double WRIST_POSITION_MAX = 0.45;

    public static final double WRIST_HOLD_POWER = 0.15;
  }
  
  public final class GrabberConstants {
    public static final int GRABBER_CANID = 7;
    public static final int GRABBER_LIMIT_SWITCH_ID = 2;
    /**when the grabber limit switch is triggered, the encoder knows it is at this value and resets, measured in motor rotations */
    public static final int GRABBER_POSITION_AT_LIMIT_SWITCH = 50;
    /** the position of the motor when the grabber is open, measured in motor rotations */
    public static final int GRABBER_POSITION_OPEN = 100;
    /** the position of the motor when the grabber is closed, measured in motor rotations */
    public static final int GRABBER_POSITION_CLOSED = 0;
    /** the dead band tolerance for the grabber, measured in motor rotations */
    public static final double GRABBER_ENCODER_DEADBAND = 0.4;

    public static final double GRABBER_POWER_OUT = 0.25;
    public static final double GRABBER_POWER_IN = -0.25;
    public static final double GRABBER_LOW_POWER_IN = -0.15;
    public static final int SMART_CURRENT_LIMIT = 30;

    public static final double GRABBER_MAX_AT_FULL_ARM = -2;
    public static final double GRABBER_GOOD_ENOUGH_SQUEEZE = 0.3; //when the triggers are substantially pressed 

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