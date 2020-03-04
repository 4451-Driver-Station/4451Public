/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTableInstance;

public class ShuffleBoardSubsystem extends SubsystemBase {

	private String mainTabName;

	NetworkTableInstance defaultInstance;

  /**
   * Creates a new ShuffleBoardSubsystem.
   */
  public ShuffleBoardSubsystem(String tab) {
		mainTabName = tab;
		defaultInstance = NetworkTableInstance.getDefault();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
	}
	
	public void put(String varName, double value) {
		defaultInstance.getTable(mainTabName).getEntry(varName).setNumber(value);
		
	}

	public void put(String tabName, String varName, double value) {
		defaultInstance.getTable(tabName).getEntry(varName).setNumber(value);
		
	}

}