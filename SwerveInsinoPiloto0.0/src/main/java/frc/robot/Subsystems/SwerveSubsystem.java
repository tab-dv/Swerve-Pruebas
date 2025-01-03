package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SwerveSubsystem extends SwerveDrivetrain implements Subsystem{

    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants[] modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);

    //Configuracion de PID motores de direccion
    Slot0Configs steerGains = new Slot0Configs()
   .withKP(100)
   .withKI(0)
   .withKD(0.2)
   .withKS(0)
   .withKV(1.5)
   .withKA(0);

    //Configuracion de PID motores de potencia
    Slot0Configs driveGains = new Slot0Configs()
   .withKP(3)
   .withKI(0)
   .withKD(0)
   .withKS(0)
   .withKV(0)
   .withKA(0);

    //Establece tipo de ciclo de la variable de direccion (Voltaje)
    ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;

    //Establece tipo de ciclo de la variable de potencia (Corriente)
    ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;

    //Limite de corriente (al arranque)
    double kSlipCurrentA = 150.0;

    //Limite de corrienre (uso normal)
    TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
    .withCurrentLimits(new CurrentLimitsConfigs()
    .withStatorCurrentLimit(60)
    .withStatorCurrentLimitEnable(true));

    //Configuracion de CANconder
    CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();

    Pigeon2Configuration pigeonConfigs = null;

    //Velocidad con carga maxima de la bateria (m/s)
    double kSpeedAt12VoltsMps = 4.73;

    //Relacion de transmision
    double kCoupleRatio = 3.5;

    //Radio de engranes (potencia y direccion)
    double kDriveGearRatio = 6.75;
    double kSteerGearRatio = 15.43;

    //Radio de la llanta
    double kWheelRadiusInches = 2;

    //Canales de sañal (CAN)
    String kCANbusName = "drivetrain";
    int kPigeonId = 1;

    //Inercia de los motores
    double kSteerInertia = 0.00001;
    double kDriveInertia = 0.001;
    
    //Voltaje necesario para superar la friccion
    double kSteerFrictionVoltage = 0.25;
    double kDriveFrictionVoltage = 0.25;

    //Constantes de odometria swerve
    SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
      .withCANbusName(kCANbusName)
      .withPigeon2Id(kPigeonId)
      .withPigeon2Configs(pigeonConfigs);

    //Configuracion de la swerve
    SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
      .withDriveMotorGearRatio(kDriveGearRatio)
      .withSteerMotorGearRatio(kSteerGearRatio)
      .withWheelRadius(kWheelRadiusInches)
      .withSlipCurrent(kSlipCurrentA)
      .withSteerMotorGains(steerGains)
      .withDriveMotorGains(driveGains)
      .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
      .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
      .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
      .withSteerInertia(kSteerInertia)
      .withDriveInertia(kDriveInertia)
      .withSteerFrictionVoltage(kSteerFrictionVoltage)
      .withDriveFrictionVoltage(kDriveFrictionVoltage)
      .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
      .withCouplingGearRatio(kCoupleRatio)
      .withSteerMotorInverted(false)
      .withDriveMotorInitialConfigs(driveInitialConfigs)
      .withSteerMotorInitialConfigs(steerInitialConfigs)
      .withCANcoderInitialConfigs(cancoderInitialConfigs);

int kFrontLeftDriveMotorId = 1;
int kFrontLeftSteerMotorId = 0;
int kFrontLeftEncoderId = 0;
double kFrontLeftEncoderOffset = -0.75;

double kFrontLeftXPosInches = 10.5;
double kFrontLeftYPosInches = 10.5;


int kFrontRightDriveMotorId = 3;
int kFrontRightSteerMotorId = 2;
int kFrontRightEncoderId = 1;
double kFrontRightEncoderOffset = -0.75;

double kFrontRightXPosInches = 10.5;
double kFrontRightYPosInches = -10.5;


int kBackLeftDriveMotorId = 5;
int kBackLeftSteerMotorId = 4;
int kBackLeftEncoderId = 2;
double kBackLeftEncoderOffset = -0.75;

double kBackLeftXPosInches = -10.5;
double kBackLeftYPosInches = 10.5;


int kBackRightDriveMotorId = 7;
int kBackRightSteerMotorId = 6;
int kBackRightEncoderId = 3;
double kBackRightEncoderOffset = -0.75;

double kBackRightXPosInches = -10.5;
double kBackRightYPosInches = -10.5;


    SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
      kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, 
      kFrontLeftEncoderId, kFrontLeftEncoderOffset, 
      Units.inchesToMeters(kFrontLeftXPosInches), 
      Units.inchesToMeters(kFrontLeftYPosInches), 
      false);

    SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
      kFrontRightSteerMotorId, kFrontRightDriveMotorId, 
      kFrontRightEncoderId, kFrontRightEncoderOffset, 
      Units.inchesToMeters(kFrontRightXPosInches), 
      Units.inchesToMeters(kFrontRightYPosInches), 
      true);

    SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
      kBackLeftSteerMotorId, 
      kBackLeftDriveMotorId, 
      kBackLeftEncoderId, 
      kBackLeftEncoderOffset, 
      Units.inchesToMeters(kBackLeftXPosInches), 
      Units.inchesToMeters(kBackLeftYPosInches), 
      false);

    SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
      kBackRightSteerMotorId, 
      kBackRightDriveMotorId, 
      kBackRightEncoderId, 
      kBackRightEncoderOffset, 
      Units.inchesToMeters(kBackRightXPosInches), 
      Units.inchesToMeters(kBackRightYPosInches), 
      true);

    CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(
        DrivetrainConstants, 
        FrontLeft,
        FrontRight, 
        BackLeft, 
        BackRight);
    }
}

me van a secuertrar le voy a cortar la pelona a mac