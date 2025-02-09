package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Colors;
import frc.robot.Constants.GameConstants;

public class Arm extends SubsystemBase{
    private final SparkMax rightMotor = new SparkMax(ArmConstants.rightMotorID, MotorType.kBrushless);
    private final SparkMax leftMotor = new SparkMax(ArmConstants.leftMotorID, MotorType.kBrushless);

    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

    private boolean isArmReady = false;

    private final DigitalInput armLimitSwitch = new DigitalInput(2);

    public PIDController armPID = new PIDController(ArmConstants.kP,0,0);

    public Arm(){
        //Default coast to raise arm manually
        setIdleMode(IdleMode.kCoast);

        // leftMotor.setSmartCurrentLimit(30);
        // rightMotor.setSmartCurrentLimit(30);

        // set in idlemode
        // rightMotor.setInverted(true);
    }

    /**
     * @param percent should be negative/positive to move the arm up/down.
     */
    public void runArm(double percent){
        if(!armLimitSwitch.get()){ //If the arm isn't fully down operate regularly
            leftMotor.set(percent);
            rightMotor.set(percent);
        } else { //If the arm is down:
            if(percent<0){ //If they want to move the arm up, allow it.
                leftMotor.set(percent);
                rightMotor.set(percent);
            } else { //Don't let them bring the arm back down
                leftMotor.set(0);
                rightMotor.set(0);
            }
        }
    }

    public void setIdleMode(IdleMode mode){
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.idleMode(mode);
        leftConfig.inverted(false);
        leftConfig.follow(ArmConstants.rightMotorID, true);
        leftMotor.configure(leftConfig, null, null);
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(mode);
        leftConfig.inverted(true);
        rightMotor.configure(rightConfig, null, null);

        // leftMotor.setIdleMode(mode);
        // rightMotor.setIdleMode(mode);
    }

    public void resetEncoders(){
        rightEncoder.setPosition(0);
        leftEncoder.setPosition(0);
    }

    public boolean getArmLimitSwitch(){
        return armLimitSwitch.get();
    }

    /**
     * Uses the right encoder.
     * @return armPosition (double)
     */
    public double getArmPosition(){
        return rightEncoder.getPosition();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("ARM ENCODER", getArmPosition());
        SmartDashboard.putBoolean("ARM DOWN", armLimitSwitch.get());

        if(getArmLimitSwitch()){
            resetEncoders();
        }

        if(RobotContainer.gameState==GameConstants.Robot){
            if(!isArmReady && getArmPosition() < ArmConstants.armStartingPos){
                setIdleMode(IdleMode.kBrake);
                RobotContainer.led.setLEDColor(Colors.green);
                isArmReady = true;
            }
        }
    }
}