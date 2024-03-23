// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    CANSparkMax m_intakeMotor = new CANSparkMax(21, MotorType.kBrushless);

    private boolean m_intakeRunning = false;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {

        // Set motor to factory defaults
        m_intakeMotor.restoreFactoryDefaults();

        // Invert motor?
        m_intakeMotor.setInverted(true);

        // Set motor to Brake
        m_intakeMotor.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void periodic() {}

    /**
     * 
     * @param speed speed to set intake motor at (-1,1)
     */
    public void runIntake(double speed) {
        m_intakeMotor.set(speed);
        m_intakeRunning = true;
    }

    public void stopIntake() {
        m_intakeMotor.set(0);
        m_intakeRunning = false;
    }

    public boolean isIntakeRunning() {
        return m_intakeRunning;
    }
    
    /*
     * Command Factories
     */
    public Command runIntakeCommand() {
        return new RunCommand(() -> this.runIntake(0.6), this);
    }

    public Command stopIntakeCommand() {
        return new InstantCommand(() -> this.stopIntake(), this);
    }

    public Command ejectIntakeCommand() {
        return startEnd(() -> this.runIntake(-0.6)
                        , () -> this.runIntake(0));
    }

}
