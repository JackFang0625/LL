// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;
/**
 *  Intake a Note
 * - Run Intake & Stage
 * - Stop everything when a Note is in the stage
 */
public class intakeNote extends Command {

    IntakeSubsystem m_intakeSubsystem;
    TransferSubsystem m_transferSubsystem;
    boolean m_isDone;

    /** Constructor - Creates a new intakeNote */
    public intakeNote(IntakeSubsystem intakeSub, TransferSubsystem transferSub) {

        m_intakeSubsystem = intakeSub;
        m_transferSubsystem = transferSub;
        addRequirements(intakeSub, transferSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_isDone = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Run the Stage until a Note is inside
        if (!m_transferSubsystem.isNoteInStage()) {
            m_transferSubsystem.runStage(0.8);
        } else {
            m_transferSubsystem.stopStage();
            m_isDone = true;
        }
        // Turn on the Intake
        m_intakeSubsystem.runIntake(0.6);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Turn off the Stage and Intake
        m_transferSubsystem.stopStage();
        m_intakeSubsystem.stopIntake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isDone;
    }
}
