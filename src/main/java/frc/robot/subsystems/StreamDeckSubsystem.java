// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StreamDeckSubsystem extends SubsystemBase {
  /** Creates a new StreamDeckSubsystem. */
  private NetworkTableInstance ntinst;

  private NetworkTable table;
  int listenerHandle;


  private StringEntry autoEntry;
  private StringEntry elevStateEntry;


  public StreamDeckSubsystem() {
    ntinst = NetworkTableInstance.getDefault();
    ntinst.setServer("localhost"); // We are running network tables on the same computer as the robot code to avoid bandwidth issues during comp
    // ntinst.setServer("127.0.0.1");
    table = ntinst.getTable("StreamDeck");
    ntinst.removeListener(0);

    // button0Entry = table.getBooleanTopic("0").getEntry(false);
    // selectedProgramEntry = table.getStringTopic("SelectedProgramString2").getEntry("");

    getEntries();

    clear();
  }

  private void getEntries(){
    autoEntry = table.getStringTopic("auto").getEntry("");
    elevStateEntry = table.getStringTopic("elevState").getEntry("");
  }

  public String getElevStateEntry(){
    String elevStateVal = elevStateEntry.get();
    try{
      SmartDashboard.putString("Streamdeck_elevState", elevStateVal);
    }
    catch(Exception e){
      
    }
    return elevStateVal;
  }

  public String getAutoEntry(){
    String autoVal = autoEntry.get();

    try{
      SmartDashboard.putString("StreamDeck_autoEntry", autoVal);
    }
    catch(Exception e){
      
    }
    return autoVal;
  }

  public void clear(){
    for(String index: table.getKeys()){
      NetworkTableEntry entry = table.getEntry(index);
      entry.clearPersistent();
      entry.setDefaultValue(false);
    }
  }

  @Override
  public void periodic() {

  }
}
