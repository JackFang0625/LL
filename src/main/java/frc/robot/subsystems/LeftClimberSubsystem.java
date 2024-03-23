package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LeftClimberSubsystem extends SubsystemBase {
    private final CANSparkMax leftClimberMotor = new CANSparkMax(28, MotorType.kBrushless);

    public LeftClimberSubsystem() {
        leftClimberMotor.restoreFactoryDefaults();

        leftClimberMotor.setIdleMode(IdleMode.kBrake);

        leftClimberMotor.setInverted(true);
    }

    public Command runCommand(double value) {
        return startEnd(() -> leftClimberMotor.set(value)
                        , () -> leftClimberMotor.set(0));
    }
}