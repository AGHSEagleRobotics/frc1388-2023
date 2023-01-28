// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Add your docs here.
 */
public class Air extends SubsystemBase {

    private final double k_transducerMaxPSI = 157.0;       // determined empirically; spec = 150 psi
    private final double k_fullRange = (k_transducerMaxPSI / 4.0 * 5.0);  // (psi / Vrange * Vfullscale) - see datasheet
    private final double k_offset = -21;      // PSI - empirical

    private boolean lastCompressorOn;           // record compressor status

    private Timer timer;
    private double loggingPeriod = 2;           // log info this often

    private int minClimbPressure = 80;          // Singal that it's ok to climb at this pressure

    private Compressor compressor;
    private AnalogPotentiometer airPressure;
    private DriverStation driverStation;

    // Constructor
    public Air() {
        // create objects
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        airPressure = new AnalogPotentiometer(1, k_fullRange, k_offset);

        timer = new Timer();
        timer.start();

        lastCompressorOn = false;
    }

    
    @Override
    public void periodic() {
        int pressure = (int) getPressure();
        boolean compressorOn = compressor.isEnabled();

        
        lastCompressorOn = compressorOn;
    }
        // Send info to dashboard
    //     SmartDashboard.putBoolean("Compressor Status", compressorOn);
    //     SmartDashboard.putNumber("Air Pressure", pressure);
    //     SmartDashboard.putBoolean("Ok To Climb", (pressure >= minClimbPressure));
    // }

    public double getPressure() {
        return airPressure.get();
    }
}