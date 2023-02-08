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

    //public static final double GAINS_VELOCITY_F = 0.047;
    public static final double GAINS_VELOCITY_F = 0.05;
    // public static final double GAINS_VELOCITY_P = 0.25; // using motor for testing
    public static final double GAINS_VELOCITY_P = 0.05; // using motor for testing
    public static final double GAINS_VELOCITY_I = 0.0005;
    public static final double GAINS_VELOCITY_D = 0;
    public static final int PID_IDX = 0;
  } 

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
