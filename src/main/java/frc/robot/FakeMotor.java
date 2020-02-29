package frc.robot;

//import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class FakeMotor extends WPI_TalonFX{

int FakePosition=0;


    public FakeMotor(int deviceNumber) {
        super(deviceNumber);
       
    }

    @Override
    public int getSelectedSensorPosition() {
        FakePosition = FakePosition+(int)(get()*1000);
        return FakePosition;

    }
  
}