package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TelescopingAmpConstants;

public class TelescopingAmpTestSubsystem extends SubsystemBase{
    private CANSparkMax telescopingArmMotor;
    private CANSparkMax topFingerMotor;
    private CANSparkMax lowerFingerMotor;

    public TelescopingAmpTestSubsystem() {
        telescopingArmMotor = new CANSparkMax(TelescopingAmpConstants.kTelescopeID, MotorType.kBrushless);
        topFingerMotor = new CANSparkMax(TelescopingAmpConstants.kTopFingerID, MotorType.kBrushless);
        lowerFingerMotor = new CANSparkMax(TelescopingAmpConstants.kLowerFingerID, MotorType.kBrushless);
    }

    public void setTopFinger(double power) {
        topFingerMotor.set(power);
    }

    public void setLowerFinger(double power) {
        lowerFingerMotor.set(power);
    }

    
    public void handIntake(double power) {
        setTopFinger(power);
        setLowerFinger(-power);
    }

    public void stopIntake() {
        setTopFinger(0);
        setLowerFinger(0);
    }
    public void setTelescopePower(double power) {
        telescopingArmMotor.set(power);
    }

    public void stopTelescope() {
        telescopingArmMotor.set(0);
    }

    public Command getMoveTelescopeCommand(DoubleSupplier powerLamda) {
        return this.runEnd(() -> {
            setTelescopePower(powerLamda.getAsDouble());
        }, () -> {
            stopTelescope();
        });
    }

    public Command getMoveTelescopeCommand(double power) {
        return this.startEnd(() -> {
            setTelescopePower(power);
        }, () -> {
            stopTelescope();
        });
    }

    public Command getRunTopFingerCommand(double power) {
        return this.startEnd(() -> {
            setTopFinger(power);
        }, () -> {
            setTopFinger(0);
        });
    }

    public Command getRunLowerFingerCommand(double power) {
        return this.runEnd(() -> {
            setLowerFinger(power);
        }, () -> {
            setLowerFinger(0);
        });
    }

    public Command getRunTopFingerCommand(DoubleSupplier powerSupplier) {
        return this.runEnd(() -> {
            setTopFinger(powerSupplier.getAsDouble());
        }, () -> {
            setTopFinger(0);
        });
    }

    public Command getRunLowerFingerCommand(DoubleSupplier powerSupplier) {
        return this.runEnd(() -> {
            setLowerFinger(powerSupplier.getAsDouble());
        }, () -> {
            setLowerFinger(0);
        });
    }

    public Command getRotateNoteCommand(double power) {
        return this.startEnd(() -> {
            if(power>0)
            {
                setTopFinger(power);
                setLowerFinger(TelescopingAmpConstants.kRotatingOffset*-power);
            }
            else {
                setTopFinger(TelescopingAmpConstants.kRotatingOffset*power);
                setLowerFinger(-power);
            }
        }, () -> {
            stopIntake();
        });
    }

    public Command getIntakeNoteCommand() {
        return this.startEnd(() -> {
            setTopFinger(TelescopingAmpConstants.kIntakeSpeed);
            setLowerFinger(TelescopingAmpConstants.kIntakeSpeed);
        }, () -> {
            stopIntake();
        });
    }

    public Command getOuttakeNoteCommand() {
        return this.startEnd(() -> {
            setTopFinger(TelescopingAmpConstants.kOuttakeSpeed);
            setLowerFinger(TelescopingAmpConstants.kOuttakeSpeed);
        }, () -> {
            stopIntake();
        });
    }
}
