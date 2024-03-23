package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransferSubsystem extends SubsystemBase {
    private final CANSparkMax TransferMotor = new CANSparkMax(27, MotorType.kBrushless);
    DigitalInput infraredSensor = new DigitalInput(0);

    boolean m_noteInStage = false;
    boolean m_stageRunning = false;

    public TransferSubsystem() {
        TransferMotor.restoreFactoryDefaults();
        TransferMotor.setIdleMode(IdleMode.kCoast);
        TransferMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        m_noteInStage = infraredSensor.get() ? false : true;
        SmartDashboard.putBoolean("Note In Stage?", m_noteInStage);
    }

    public void runStage(double speed) {
        TransferMotor.set(speed);
        m_stageRunning = true;
    }

    public void stopStage() {
        TransferMotor.set(0);
        m_stageRunning = false;
    }

    public boolean isNoteInStage() {
        return m_noteInStage;
    }

    public boolean isStageRunning() {
        return m_stageRunning;
    }

    /*
     * Command Factories
     */

    // Pass the Note to the Shooter
    public Command feedNote2ShooterCommand() {

        if (isNoteInStage()) {
            return new RunCommand(() -> this.runStage(1), this)
                .until(() -> !isNoteInStage()) // run until there is NOT a Note in the Stage
                .andThen(() -> this.stopStage());

        } else {
            // If there's no note to start, run with timeout
            return new RunCommand(() -> this.runStage(1), this)
                .withTimeout(1.5) // run for 1.5 seconds
                .andThen(() -> this.stopStage());
        } 
    }

    public Command feedStageCommand() {

        return new RunCommand(() -> this.runStage(0.8), this)
                .until(() -> isNoteInStage()) // run until there is NOT a Note in the Stage
                .andThen(() -> this.stopStage()); 
    }

}