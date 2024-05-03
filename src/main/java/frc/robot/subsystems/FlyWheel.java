package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignal.SignalMeasurement;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class FlyWheel extends SubsystemBase{
    private final TalonFX leftWheel;
    private final TalonFX rightWheel;

    private final int leftWheelID = 14; //ID's of TalonFX through CANBUS
    private final int rightWheelID = 1; //ID's of TalonFX through CANBUS

    private volatile double targetSpeed;

    private final double MAX_SPEED = 511.998;
    private final double MIN_SPEED = -512;

    private boolean wheelTargetHit = false;

    private final double INPUT_THRESHOLD = 0.001;

    public FlyWheel(){
        leftWheel = new TalonFX(leftWheelID);
        rightWheel = new TalonFX(rightWheelID);

        leftWheel.setNeutralMode(NeutralModeValue.Coast);
        rightWheel.setNeutralMode(NeutralModeValue.Coast);

        

    }

    public void shoot(double power)
    {

        /*deadband
        if(power<0.05){
            power = 0;
        }*/
        
        leftWheel.set(-power*0.8);
        rightWheel.set(power*0.81);//puts spin on ring
    }

    public void stop()
    {
        leftWheel.set(0);
        rightWheel.set(0);
    }

    public double getLeftSpeed()
    {
        return leftWheel.getVelocity().getValueAsDouble(); //range {-512, 511.998} rotations per second
    }

    public double getRightSpeed()
    {
        return rightWheel.getVelocity().getValueAsDouble(); //range {-512, 511.998} rotations per second
    }

    public double getPivotTargetSpeed()
    {
        return targetSpeed;
    }

    public void setPivotTargetSpeed(double speed)
    {
        targetSpeed = speed;
    }

   

}
