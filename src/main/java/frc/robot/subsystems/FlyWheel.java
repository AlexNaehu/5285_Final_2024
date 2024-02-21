package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class FlyWheel extends SubsystemBase{
    private final CANSparkMax leftWheel;
    private final CANSparkMax rightWheel;

    private final int leftWheelID = 24;
    private final int rightWheelID = 21;

    public FlyWheel(){
        leftWheel = new CANSparkMax(leftWheelID, MotorType.kBrushless);
        rightWheel = new CANSparkMax(rightWheelID, MotorType.kBrushless);

        leftWheel.setIdleMode(IdleMode.kCoast);
        rightWheel.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
    }

    public void shoot(double power){
        leftWheel.set(-power);
        rightWheel.set(power);
    }

    public void stop(){
        leftWheel.set(0);
        rightWheel.set(0);
    }

}
