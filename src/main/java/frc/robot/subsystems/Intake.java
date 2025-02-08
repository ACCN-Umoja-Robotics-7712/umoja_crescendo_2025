package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Colors;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    public static final SparkMax leftWheel = new SparkMax(IntakeConstants.leftWheelID, MotorType.kBrushless);
    public static final SparkMax rightWheel = new SparkMax(IntakeConstants.rightWheelID, MotorType.kBrushless);
    public static final SparkMax frontWheel = new SparkMax(IntakeConstants.frontWheelID, MotorType.kBrushless);

    public static final DigitalInput intakeSensor = new DigitalInput(9);

    public boolean hasNote, isShooting = false;
    public boolean intakeState;
    private Color intakeColor;

    /**
     * Runs during RobotInit(). Define idlemodes, current limits, etc.
     */
    public Intake() {
        // leftWheel.setIdleMode(IdleMode.kCoast);
        // rightWheel.setIdleMode(IdleMode.kCoast);
        // frontWheel.setIdleMode(IdleMode.kCoast);

        // rightWheel.setInverted(true);
        // leftWheel.setInverted(true);
    }

    /**
     * @param percent should be positive to intake.
     */
    public void runIntake(double percent){
        frontWheel.set(percent);
    }

    /**
     * @param percent should be negative to shoot out.
     */
    public void runShooter(double percent){
        rightWheel.set(percent);
        leftWheel.set(percent);
    }

    /**
     * Stops all intake & shooting motors.
     */
    public void stop(){
        runIntake(0);
        runShooter(0);
    }

    public boolean getIntakeSwitch(){
        return !intakeSensor.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("INTAKE SWITCH", intakeState);

        intakeState = getIntakeSwitch();

        if(hasNote!=intakeState){
            if(RobotContainer.gameState==GameConstants.TeleOp){
                if(intakeState){
                    intakeColor = Colors.green;
                } else {
                    intakeColor = Colors.white;
                }                
                RobotContainer.led.setLEDColor(intakeColor);
            }
            
            hasNote = !hasNote;
        }
    }
}