// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.Limelight;
import frc.robot.subsystems.ArmSubsystem;

public class autoAlign extends Command {

    ArmSubsystem m_armSubsystem;
    Limelight m_limelightSubsystem;
    boolean m_isDone;

    /** Constructor - Creates a new intakeNote */
    public autoAlign(ArmSubsystem armSub, Limelight limelightSub) {

        m_armSubsystem = armSub;
        m_limelightSubsystem = limelightSub;
        addRequirements(armSub, limelightSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_isDone = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double tagSize = m_limelightSubsystem.getTA();
        double autoAglinGoal = 0.1-((tagSize-0.1)/5);
        SmartDashboard.putNumber("ta", tagSize);
        SmartDashboard.putNumber("autoAglin", autoAglinGoal);
        m_armSubsystem.pidCommand(autoAglinGoal);
/*
        if(tagSize >= 0.1 && tagSize <= 0.6){
            m_armSubsystem.pidCommand(autoAglinGoal);
            SmartDashboard.putBoolean("run", true);
        }else{
            SmartDashboard.putBoolean("run", false);
            m_armSubsystem.stopArm();
            m_isDone = true;
        }
*/
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Turn off the arm
        SmartDashboard.putBoolean("run", false);
        m_armSubsystem.stopArm();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isDone;
    }
}
