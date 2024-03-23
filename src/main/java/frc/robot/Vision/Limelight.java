// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;

public class Limelight extends SubsystemBase {
    CommandSwerveDrivetrain m_drivetrain;
    private String ll = "limelight-middle";
    private Boolean enable = true;
    private Boolean hasTarget = false;

    /** Creates a new Limelight. */
    public Limelight(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        if (enable) {
            if (NetworkTableInstance.getDefault().getTable(ll).getEntry("tv").getBoolean(false)) {
                if (NetworkTableInstance.getDefault().getTable(ll).getEntry("ta").getDouble(0) > .1) {
                    hasTarget = true;
                } else {
                    hasTarget = false;
                }
            }
            SmartDashboard.putBoolean("Limelight has note detected", hasTarget);
        }
    }

    public double getTA(){
        return NetworkTableInstance.getDefault().getTable(ll).getEntry("ta").getDouble(0);
    }

    public void useLimelight(boolean enable) {
        this.enable = enable;
    }

    public boolean hasTarget() {
        return hasTarget;
    }
}