package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    
    private final CANSparkMax leftHook;
    private final CANSparkMax rightHook;

    private final int leftHookID = 19;
    private final int rightHookID = 20;

    public Climber(){
        leftHook = new CANSparkMax(leftHookID, MotorType.kBrushless);
        rightHook = new CANSparkMax(rightHookID, MotorType.kBrushless);

        leftHook.setIdleMode(IdleMode.kBrake);
        rightHook.setIdleMode(IdleMode.kBrake);

        leftHook.setSmartCurrentLimit(15);
        rightHook.setSmartCurrentLimit(15);
    }

    public void climb(double power){
        leftHook.set(-power);
        rightHook.set(power);
    }

    public void stop(){
        leftHook.set(0);
        rightHook.set(0);
    }


}
