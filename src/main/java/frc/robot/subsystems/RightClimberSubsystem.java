package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RightClimberSubsystem extends SubsystemBase {
    private final CANSparkMax rightClimberMotor = new CANSparkMax(29, MotorType.kBrushless);

    public RightClimberSubsystem() {
        rightClimberMotor.restoreFactoryDefaults();

        rightClimberMotor.setIdleMode(IdleMode.kBrake);

        rightClimberMotor.setInverted(false);
    }

    public Command runCommand(double percent) {
        return startEnd(() -> rightClimberMotor.set(percent)
                        , () -> rightClimberMotor.set(0));
    }
}