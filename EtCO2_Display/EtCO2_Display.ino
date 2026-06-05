void alarmCount(float co2mmhg) {
  
  // Evaluate the real-time blocking condition explicitly every frame
  // The line is blocked ONLY if communication is healthy (valid) AND flow is beneath minimums
  bool currentlyBlocked = (g_flowValid && g_flowSLPM < FLOW_THRESHOLD_SLPM);

  // 1. STATIC LINE BLOCKED MANAGEMENT (No flashing, No buzzer, Draw-Once logic)
  if (currentlyBlocked) {
    if (!wasLineBlocked) { 
      // Line just blocked! Draw the warning ONCE to prevent flickering
      my_lcd.fillRect(90, 85, 300, 150, RED);
      my_lcd.setTextColor(YELLOW);
      my_lcd.setTextSize(4);
      my_lcd.drawString("LINE",    200, 125);
      my_lcd.drawString("BLOCKED", 165, 175);
      my_lcd.setTextColor(WHITE);
      wasLineBlocked = true;
    }
    return; // Freeze execution here so apnea alarms don't fight for the screen
  } 
  
  // 2. CLEAR THE WARNING IMMEDIATELY WHEN UNBLOCKED
  if (!currentlyBlocked && wasLineBlocked) {
    // Line just cleared! Wipe the box and instantly rebuild the underlying GUI layout
    my_lcd.fillRect(90, 85, 300, 150, BLACK);
    FailResult fails = checkFails();
    drawGUI(1, 1, edit, mode, fails.flowFail, fails.tempFail, fails.humFail, fails.battFail);
    updateEtCO2(EtCO2);
    wasLineBlocked = false; 
  }

  // 3. YOUR ORIGINAL APNEA LOGIC UNTOUCHED BELOW
  if (co2mmhg < 3) {
    if (!apneaTracking) {
      apneaTracking = true;
      apneaStartT = millis();   
    }
    if (millis() - apneaStartT >= 15000UL && alarmState == ALARM_OFF) {
      alarmState = ALARM_APNEA;
      alarmVisible = false;
      alarmFlashT = millis();
      alarmAcked = false;
    }
    if(millis() - apneaStartT >= 20000UL) {
      apneaYes = 1;
    } else {
      apneaYes = 0;
    }
  } else { 
    if (alarmState == ALARM_APNEA) {
      digitalWrite(ALARM_PIN, LOW);                        
      my_lcd.fillRect(90, 85, 300, 150, BLACK);     
      FailResult fails = checkFails();
      drawGUI(1, 1, edit, mode, fails.flowFail, fails.tempFail, fails.humFail, fails.battFail);                                
    }
    apneaTracking = false;
    apneaStartT = 0;
    alarmState = ALARM_OFF;
    alarmVisible  = false;
    alarmAcked = false;
  }

  if(!status) {
    if (alarmState == ALARM_APNEA) {
      digitalWrite(ALARM_PIN, LOW);                        
      my_lcd.fillRect(90, 85, 300, 150, BLACK);     
      FailResult fails = checkFails();
      drawGUI(1, 1, edit, mode, fails.flowFail, fails.tempFail, fails.humFail, fails.battFail);                                
    }
    apneaTracking = false;
    apneaStartT = 0;
    alarmState = ALARM_OFF;
    alarmVisible  = false;
    alarmAcked = false;
    apneaYes = 0;
  }
}
