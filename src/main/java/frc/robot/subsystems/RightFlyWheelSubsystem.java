package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RightFlyWheelSubsystem extends SubsystemBase {
    private final CANSparkMax rightFlyWheelMotor = new CANSparkMax(26, MotorType.kBrushless);

    private final RelativeEncoder rightFlyWheelEncoder = rightFlyWheelMotor.getEncoder();

    private final SparkPIDController m_pidController = rightFlyWheelMotor.getPIDController();

    private final double kP = 0, kI = 0, kD = 0, kIz = 0, kFF = 0.000179
                            , kMaxOutput = 1, kMinOutput = -1;

    private int idealVolecity = 0;

    public void periodic() {
        SmartDashboard.putNumber("rightFlyWheel", rightFlyWheelEncoder.getVelocity());
    }

    public RightFlyWheelSubsystem() {
        rightFlyWheelMotor.restoreFactoryDefaults();
        rightFlyWheelMotor.setIdleMode(IdleMode.kCoast);
        rightFlyWheelMotor.setInverted(true);

            // set PID coefficients
            m_pidController.setP(kP);
            m_pidController.setI(kI);
            m_pidController.setD(kD);
            m_pidController.setIZone(kIz);
            m_pidController.setFF(kFF);
            m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    }

    public Command pidCommand(double Velocity) {
        return startEnd( () -> m_pidController.setReference(Velocity, CANSparkMax.ControlType.kVelocity)
                        , () -> rightFlyWheelMotor.set(0));
    }
    
    public Command stopCommand() {
        return runOnce(() -> rightFlyWheelMotor.set(0));
    }

    public BooleanSupplier ifVelocity(){
        return ()-> rightFlyWheelEncoder.getVelocity() == idealVolecity;
    }
}