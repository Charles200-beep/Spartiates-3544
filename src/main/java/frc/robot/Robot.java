/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Importer les librairies
package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

//Définir les composantes
public class Robot extends TimedRobot {
  private PIDController m_pidController;
  private PIDController m_pidController2;
  private WPI_VictorSPX m_leftMotor;
  private WPI_VictorSPX m_rightMotor;
  private WPI_VictorSPX m_leftMotor2;
  private WPI_VictorSPX m_leftMotor3;
  private WPI_VictorSPX m_rightMotor2;
  private WPI_VictorSPX m_rightMotor3;
  private DifferentialDrive m_robotDrive;
  private Joystick m_stick;
  private Compressor c;
  private WPI_TalonFX m_test;
  private AHRS ahrs; 
  private static final double kP = -.075;
  private static final double kP2 = -.075;
  private static final double kI = -0.00;
  private static final double kD = -0.0;
  private static final double kD2 = -0.0;
  boolean jamaisattetint = true;
  boolean jamaisattetint2 = true;
  boolean jamaisattetintroule = false;


//Accéder aux données du limelight
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  @Override
  public void robotInit() {
    m_test = new WPI_TalonFX(01);
    m_leftMotor2 = new WPI_VictorSPX(5);
    m_leftMotor = new WPI_VictorSPX(6);
    m_leftMotor3 = new WPI_VictorSPX(7);
    m_rightMotor = new WPI_VictorSPX(9);
    m_rightMotor2 = new WPI_VictorSPX(10);
    m_rightMotor3 = new WPI_VictorSPX(8);
    m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_stick = new Joystick(0);
    ahrs = new AHRS(SPI.Port.kMXP);
    m_pidController = new PIDController(kP, kI, kD, 0.02);
    m_pidController2 = new PIDController(kP2, kI, kD2, 0.02);
    m_pidController.setSetpoint(90.0);
    m_pidController2.setSetpoint(0.0);
    m_test.setSelectedSensorPosition(0);
   
   //Faire suivre les autres moteurs
    m_leftMotor2.follow(m_leftMotor);
    m_leftMotor3.follow(m_leftMotor);
    m_rightMotor2.follow(m_rightMotor);
   m_rightMotor3.follow(m_rightMotor);

   c = new Compressor(0);}

   
//Conduire avec 'arcade drive'
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(m_stick.getY(), -m_stick.getX());

    //Allumer les compresseur
    if (m_stick.getRawButton(5)) {
      c.start();
    } else {
      c.stop();
    }

    // détecter la chaleur des moteurs
   
   double m_temperature = m_test.getTemperature();
   //changer pour modifier le maximum de temperature
  if (m_temperature>40) {
    c.start();
  } else {
    c.stop();
  }

SmartDashboard.putNumber("m_temperature", m_temperature);

  //Lire les données du limelight
double x = tx.getDouble(0.0);
double y = ty.getDouble(0.0);
double area = ta.getDouble(0.0);

//Lire les données du navX
double anglemesure = ahrs.getYaw();
double vitesseangulaire = ahrs.getRawGyroX();

double pidOut2 = m_pidController2.calculate(anglemesure);
double pidOut = m_pidController.calculate(anglemesure);


SmartDashboard.putNumber("anglemesure", anglemesure);
SmartDashboard.putNumber("vitesseangulaire", vitesseangulaire);
//Poster au smart dashboard les données du limelight
SmartDashboard.putNumber("LimelightX", x);
SmartDashboard.putNumber("LimelightY", y);
SmartDashboard.putNumber("LimelightArea", area);

SmartDashboard.putNumber("pidOut2", pidOut2);
SmartDashboard.putNumber("pidOut", pidOut);

//Système de controle automatique
if (m_stick.getRawButton(1)) {
    m_robotDrive.arcadeDrive(-0.7, -x*0.04);
  
  }
//Tourner a un angle
if (m_stick.getRawButton(2)) {
  ahrs.reset();
//faire
// m_pidController2.setSetpoint(0.0);

} else {
  
}
    
if (m_stick.getRawButton(3)) {
  //double erreur = 90.0 - anglemesure;
  m_robotDrive.arcadeDrive(0.0, pidOut);
}

if (m_stick.getRawButton(4)) {
  m_robotDrive.arcadeDrive(-0.7, pidOut2);
}
if (m_stick.getRawButton(7)) {
  m_test.setSelectedSensorPosition(0);

}
  double m_distance = m_test.getSelectedSensorPosition();
  SmartDashboard.putNumber("m_distance", m_distance);

}//Fin du teleop.periodic

public void autonomousPeriodic() {

//partie autonome
double m_distance = m_test.getSelectedSensorPosition();
SmartDashboard.putNumber("m_distance", m_distance);

if (jamaisattetint) {
  if (m_distance<5000) {
  m_test.set(0.07);
  
} else {
  m_test.set(0.0);
  jamaisattetint = false;
  jamaisattetintroule = true;
}

}
if (jamaisattetint2 & jamaisattetintroule) {
  if (m_distance>-5000) {
    m_test.set(-0.07);
    
  } else {
    m_test.set(0.0);
    jamaisattetint2 = false;
    
  }
}
// if (m_distance>-60000) {
//   m_test.set(-0.07);
// } else {m_test.set(0.0);
  
// }

//avancer


//s'aligner et tire








  }}








   