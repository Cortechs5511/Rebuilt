// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.misc;

import static frc.robot.subsystems.misc.AuxMotorConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AuxMotor extends SubsystemBase {
  private final SparkMax motor;

  public AuxMotor(boolean hardwareEnabled) {
    motor = hardwareEnabled ? new SparkMax(motorCanId, MotorType.kBrushless) : null;
  }

  public void setPercent(double output) {
    if (motor != null) {
      motor.set(output);
    }
  }

  public void stop() {
    setPercent(0.0);
  }
}
