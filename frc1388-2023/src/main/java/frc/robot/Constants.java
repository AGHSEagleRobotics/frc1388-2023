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

    private final static double COUNTS_PER_REV = 2048;
    private final static double REVS_PER_COUNT = 1 / COUNTS_PER_REV;
    private final static double WHEEL_DIAMETER_INCHES = 6.0; // make range value?
    private final static double FALCON_TO_SIMPLE_BOX_GEAR_RATIO = 1 / 4.67;
    private final static double SIMPLE_BOX_TO_WHEELS_RATIO = 12.0 / 30.0; // 12 sprockets simple box to 30 sprockets
                                                                          // wheel
    public final static double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;
    public final static double INCHES_PER_ENCODER_UNITS =
        REVS_PER_COUNT *
        FALCON_TO_SIMPLE_BOX_GEAR_RATIO *
        SIMPLE_BOX_TO_WHEELS_RATIO *
        WHEEL_CIRCUMFERENCE;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    
  }
}
