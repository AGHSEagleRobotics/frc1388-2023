// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {
  private final NetworkTableEntry m_tx;
  private double txPrev;
  private final NetworkTableEntry m_ty;
  private final NetworkTableEntry m_ta;
  private final NetworkTableEntry m_tv;
  private final NetworkTableEntry m_ts;
  private final NetworkTableEntry m_tp;
  private final NetworkTableEntry m_tshort;
  private final NetworkTableEntry m_tlong;
  private final NetworkTableEntry m_thor;
  private final NetworkTableEntry m_tvert;
  private final NetworkTableEntry m_botpos;
  private int tick = 0;


  private final NetworkTable m_table;
  /** Creates a new Limelight. */
  public Limelight() {
    m_table = NetworkTableInstance.getDefault().getTable("limelight");
    m_tx = m_table.getEntry("tx");
    m_ty = m_table.getEntry("ty");
    m_ta = m_table.getEntry("ta");
    m_tv = m_table.getEntry("tv");
    m_ts = m_table.getEntry("ts");
    m_tp = m_table.getEntry("tp");
    m_tshort = m_table.getEntry("tshort");
    m_tlong = m_table.getEntry("tlong");
    m_thor = m_table.getEntry("thor");
    m_tvert = m_table.getEntry("tvert");
    m_botpos = m_table.getEntry("botpose");


  }

  public double getTx() {
    return m_tx.getDouble(0.0);
  }

  public double getTy() {
    return m_ty.getDouble(0.0);
  }

  public double getTa() {
    return m_ta.getDouble(0.0);
  }

  public double getTv() {
    return m_tv.getDouble(0.0);
  }

  @Override
  public void periodic() {
    tick++;
    if (tick > 10)tick = 0;
    // // This method will be called once per scheduler run
    // PowerDistribution pdh = new PowerDistribution();
    // System.out.println("pdh 0: " + pdh.getCurrent(0) + " pdh 2: " + pdh.getCurrent(2));
    // System.out.println("ts: " + m_ts.getDouble(0.0));
    // System.out.println(m_tp.getDouble(0.0));
    SmartDashboard.putNumber("target area", getTv());
    SmartDashboard.putNumber("target x", getTx());
    SmartDashboard.putNumber("target y", getTy());
    SmartDashboard.putNumber("target short", m_tshort.getDouble(0.0));
    SmartDashboard.putNumber("target long", m_tlong.getDouble(0.0));
    SmartDashboard.putNumber("target hor", m_thor.getDouble(0.0));
    SmartDashboard.putNumber("target vert", m_tvert.getDouble(0.0));
    SmartDashboard.putNumber("bot position 0", m_botpos.getDoubleArray(new double[6])[0]);
    SmartDashboard.putNumber("bot position 1", m_botpos.getDoubleArray(new double[6])[1]);
    SmartDashboard.putNumber("bot position 2", m_botpos.getDoubleArray(new double[6])[2]);

  }
}
