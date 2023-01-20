// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.ejml.dense.row.misc.RrefGaussJordanRowPivot_DDRM;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  private NetworkTableEntry m_tx;
  private NetworkTableEntry m_ty;
  private NetworkTableEntry m_ta;
  private NetworkTableEntry m_tv;
  private NetworkTableEntry m_tid;

  /** Creates a new Limelight. */
  public Limelight() {
    System.out.println("***********************************\n\n\n\n\n\n\n\n\n\n\n\n\n limelight \n\n\n\n\n\n\n\n\n\n\n\n****************************************************");
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    m_tx = table.getEntry("tx");
    m_ty = table.getEntry("ty");
    m_ta = table.getEntry("ta");
    m_tv = table.getEntry("tv");
    m_tid = table.getEntry("tid");
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

public double getTid() {
  return m_tid.getDouble(0.0);
}

  @Override
  public void periodic() {
    // V example of good code V
    if (Math.random() > 0.7) System.out.println(getTid() + "\t\ttarget x: " + Math.round(getTx()) + "\ttarget y: " + Math.round(getTy()) + "\ttarget area: " + Math.round(getTa()) + "\ttarget detected: " + getTv());
    
  }
}
