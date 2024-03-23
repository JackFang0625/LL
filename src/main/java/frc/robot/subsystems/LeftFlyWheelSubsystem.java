package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LeftFlyWheelSubsystem extends SubsystemBase {
    private final CANSparkMax leftFlyWheelMotor = new CANSparkMax(25, MotorType.kBrushless);

    private final RelativeEncoder leftFlyWheelEncoder = leftFlyWheelMotor.getEncoder();

    private final SparkPIDController m_pidController = leftFlyWheelMotor.getPIDController();

    private final double kP = 0, kI = 0, kD = 0, kIz = 0, kFF = 0.000179
                            , kMaxOutput = 1, kMinOutput = -1;

    private int idealVolecity = 0;

    public void periodic() {
        SmartDashboard.putNumber("leftFlyWheel", leftFlyWheelEncoder.getVelocity());
    }

    public LeftFlyWheelSubsystem() {
        leftFlyWheelMotor.restoreFactoryDefaults();
        leftFlyWheelMotor.setIdleMode(IdleMode.kCoast);
        leftFlyWheelMotor.setInverted(false);

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    }

    public Command pidCommand(double Velocity) {
        return startEnd(() -> leftFlyWheelMotor.set(0.5)
                        , () -> leftFlyWheelMotor.set(0));
    }

    public Command stopCommand() {
        return runOnce(() -> leftFlyWheelMotor.set(0));
    }

    public BooleanSupplier ifVelocity(){
        return ()-> leftFlyWheelEncoder.getVelocity() == idealVolecity;
    }
}