package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX leftArmMotor = new TalonFX(23, "Drivetrain");
    private final TalonFX rightArmMotor = new TalonFX(24, "Drivetrain");
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    private final MotionMagicTorqueCurrentFOC m_mmReq = new MotionMagicTorqueCurrentFOC(0);

    double autoAglinGoal = 0;
    double tagSize = 0.003;
    boolean atSetPoint = false;
    double tolerance = 0.01;

    public ArmSubsystem() {

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        rightArmMotor.setControl(new Follower(leftArmMotor.getDeviceID(), true));

        /* Configure current limits */
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = 5; // 5 rotations per second cruise
        mm.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel 
        mm.MotionMagicJerk = 50;

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = 75;
        slot0.kI = 0;
        slot0.kD = 23;
        slot0.kV = 3.56;
        slot0.kS = 0.5; // Approximately 0.25V to get the mechanism moving
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = 130;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = 130;

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 180;
        /* Retry config apply up to 5 times, report if failure */

        StatusCode status1 = StatusCode.StatusCodeNotInitialized;
        StatusCode status2 = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status1 = leftArmMotor.getConfigurator().apply(cfg);
            status2 = rightArmMotor.getConfigurator().apply(cfg);
        if (status1.isOK() && status2.isOK()) break;
        }

        /* Make sure we start at 0 */
        leftArmMotor.setPosition(0);
        SmartDashboard.putNumber("ta", 0);
        SmartDashboard.putNumber("autoAglin", 0);
    }

    public void periodic(){
        SmartDashboard.putNumber("Arm Position", leftArmMotor.getPosition().getValueAsDouble());
    }

    public boolean atSetPoint(double goal){
        return Math.abs(goal-leftArmMotor.getPosition().getValueAsDouble()) < tolerance;
    }

    public void stopArm() {
        leftArmMotor.set(0);
    }

    public Command resetEncoder() {
        return runOnce( () -> leftArmMotor.setPosition(0));
    }

    public Command runCommand(double velocity) {
        return startEnd( () -> leftArmMotor.set(velocity)
                        , () -> stopArm() );
    }

    public Command stowedCommand() {
        return run( () -> leftArmMotor.setControl(m_mmReq.withPosition(0.040).withSlot(0)))
                .until( () -> atSetPoint(0.040))
                .andThen(() -> this.stopArm());
    }

    public Command speakerCommand() {
        return run( () -> leftArmMotor.setControl(m_mmReq.withPosition(0.050).withSlot(0)))
                .until( () -> atSetPoint(0.050))
                .andThen(() -> this.stopArm());
    }

    public Command sourceCommand() {
        return run( () -> leftArmMotor.setControl(m_mmReq.withPosition(0.136).withSlot(0)))
                .until( () -> atSetPoint(0.136))
                .andThen(() -> this.stopArm());
    }

    public Command ampCommand() {
        return run( () -> leftArmMotor.setControl(m_mmReq.withPosition(0.274).withSlot(0)))
                .until( () -> atSetPoint(0.274))
                .andThen(() -> this.stopArm());
    }

    public Command pidCommand(double position) {
        return run( () -> leftArmMotor.setControl(m_mmReq.withPosition(position).withSlot(0)))
                .until( () -> atSetPoint(position))
                .andThen(() -> this.stopArm());
    }
/*
    public Command autoAlignCommand() {
        autoAglinGoal = 0.1-((tagSize-0.1)/5);
        SmartDashboard.putNumber("ta", tagSize);
        SmartDashboard.putNumber("autoAglin", autoAglinGoal);
        
        if(tagSize >= 0.1 && tagSize <= 0.6){
            return startEnd( () -> leftArmMotor.setControl(m_mmReq.withPosition(autoAglinGoal).withSlot(0))
                        , () -> stopArm() );
        }else{
            return startEnd( () -> leftArmMotor.setControl(m_mmReq.withPosition(0.040).withSlot(0))
                        , () -> stopArm() );
        }
    }
*/
}
