package frc.robot;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SPI.Port;

public class FakeAHRS extends AHRS{

    float FakeYAW = (float) 0.0;
    SpeedController leftmotor;
    SpeedController rightmotor;
    

public FakeAHRS(Port kmxp) {
	}


public void YawMotors(SpeedController left, SpeedController right) {
leftmotor=left;
rightmotor=right;

}


    public double YawSpeed() {
        double leftSpeed = leftmotor.get();
        double rightSpeed = rightmotor.get();
        return -(leftSpeed+rightSpeed);


    }

    
        @Override
        public float getYaw() {
            FakeYAW = FakeYAW +(float) 0.0 + (float)YawSpeed();
            return FakeYAW;
    
        }
    
        @Override
        public void reset() {
            FakeYAW= (float) 0.0;
            
    }
      
    }