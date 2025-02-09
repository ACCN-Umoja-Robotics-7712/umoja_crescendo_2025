package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase{
    public static final SparkMax rightMotor = new SparkMax(ClimbConstants.rightMotorID, MotorType.kBrushless);
    public static final SparkMax leftMotor = new SparkMax(ClimbConstants.leftMotorID, MotorType.kBrushless);

    public static final RelativeEncoder leftEncoder = leftMotor.getEncoder();
    public static final RelativeEncoder rightEncoder = rightMotor.getEncoder();

    public static final PIDController climbPID = new PIDController(ClimbConstants.kP,0,0);

    public Climber() {
        // leftMotor.setIdleMode(IdleMode.kBrake);
        // rightMotor.setIdleMode(IdleMode.kBrake);

        // leftMotor.setInverted(true);
        // rightMotor.setInverted(true);
    }

    public void runClimber(double percent){
        rightMotor.set(percent);
        leftMotor.set(percent);
    }

    public void stop(){
        rightMotor.set(0);
        leftMotor.set(0);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();

        SmartDashboard.putNumber("CLIMBER ENCODER", rightMotor.getEncoder().getPosition());
    }
}
