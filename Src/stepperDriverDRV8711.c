#include "stepperDriverDRV8711.h"
#include "stm32f1xx_hal.h"
#include "main.h"
/// This clas provides low-level functions for reading and writing from the SPI
/// interface of a DRV8711 stepper motor controller IC.
///
/// Most users should use the HighPowerStepperDriver class, which provides a
/// higher-level interface, instead of this class.
extern SPI_HandleTypeDef hspi1;
extern uint8_t motor1_driv_slp,motor2_driv_slp;

  /// Configures this object to use the specified pin as a chip select pin.
  ///
  /// You must use a chip select pin; the DRV8711 requires it.
  void setChipSelectPin(uint8_t pin)
  {
    //csPin = pin;
    //digitalWrite(csPin, LOW);
    //pinMode(csPin, OUTPUT);
  }
	
  void selectChip1(void)
  {
    	HAL_GPIO_WritePin(SPICS_M1_GPIO_Port,SPICS_M1_Pin,GPIO_PIN_SET);
  }

  void deselectChip1(void)
  {
    	HAL_GPIO_WritePin(SPICS_M1_GPIO_Port,SPICS_M1_Pin,GPIO_PIN_RESET);   
  }
	
	  void selectChip2(void)
  {
    	HAL_GPIO_WritePin(SPICS_M2_GPIO_Port,SPICS_M2_Pin,GPIO_PIN_SET);
  }

  void deselectChip2(void)
  {
    	HAL_GPIO_WritePin(SPICS_M2_GPIO_Port,SPICS_M2_Pin,GPIO_PIN_RESET);   
  }
	
	 void selectChip(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
		 
	 }

  void deselectChip(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
		
	}
	
  /// Reads the register at the given address and returns its raw value.
  uint16_t SPIreadReg(uint8_t address,uint16_t M)
  {
    // Read/write bit and register address are the first 4 bits of the first
    // byte; data is in the remaining 4 bits of the first byte combined with
    // the second byte (12 bits total).
		uint8_t SPIBuf[2];
		uint8_t outData=0;


		outData=((address & 0x03) << 4);

		if(M==1) selectChip1();
		else if(M==2) selectChip2();
		
		HAL_SPI_Transmit (&hspi1, &address, 1, 100);  // send address
		HAL_SPI_Receive (&hspi1, SPIBuf, 2, 100);  // receive 6 bytes data
		
		HAL_SPI_Receive(&hspi1,SPIBuf,2,1000);
		
	  deselectChip1();
		deselectChip2();
	  
		
  }

  /// Reads the register at the given address and returns its raw value.
  uint16_t readReg(HPSDRegAddr address, uint16_t M)
  {
    return SPIreadReg((uint8_t)address, M);
  }

  /// Writes the specified value to a register.
  void SPIwriteReg(uint8_t address, uint16_t value, uint16_t M)
  {
    // Read/write bit and register address are the first 4 bits of the first
    // byte; data is in the remaining 4 bits of the first byte combined with
    // the second byte (12 bits total).
		uint8_t SPIBuf[2];
		//uint16_t writeVal=0;
	
		//writeVal=((address & 0x03) << 12) | (value & 0x0FFF);
		SPIBuf[0]=(uint8_t)(((value & 0x0F00)>>8)|((address & 0x03) << 4));
		SPIBuf[1]=(uint8_t)((value & 0x00FF));
		if(M==1) selectChip1();
		else if(M==2) selectChip2();
    //selectChip();
		//HAL_SPI_Transmit(&hspi1,SPIBuf,2,500);
		HAL_SPI_Transmit(&hspi1,SPIBuf,2,1000);		
    // The CS line must go low after writing for the value to actually take
    // effect.
		deselectChip1();
		deselectChip2();
    //deselectChip();
  }

  /// Writes the specified value to a register.
  void writeReg(HPSDRegAddr address, uint16_t value, uint16_t M)
  {
    SPIwriteReg((uint8_t)address, value , M);
  }

  /// Re-writes the cached settings stored in this class to the device.
  ///
  /// You should not normally need to call this function because settings are
  /// written to the device whenever they are changed.  However, if
  /// verifySettings() returns false (due to a power interruption, for
  /// instance), then you could use applySettings() to get the device's settings
  /// back into the desired state.
  void applySettings(StepperDRVConfig s)
  {
    writeTORQUE(s);
    writeOFF(s);
    writeBLANK(s);
    writeDECAY(s);
    writeDRIVE(s);
    writeSTALL(s);

    // CTRL is written last because it contains the ENBL bit, and we want to try
    // to have all the other settings correct first.  (For example, TORQUE
    // defaults to 0xFF (the maximum value), so it would be better to set a more
    // appropriate value if necessary before enabling the motor.)
    writeCTRL(s);
  }


  /// Changes all of the driver's settings back to their default values.
  ///
  /// It is good to call this near the beginning of your program to ensure that
  /// there are no settings left over from an earlier time that might affect the
  /// operation of the driver.
  void resetSettings(StepperDRVConfig s)
  {
    applySettings(s);
  }

  /// Reads back the SPI configuration registers from the device and verifies
  /// that they are equal to the cached copies stored in this class.
  ///
  /// This can be used to verify that the driver is powered on and has not lost
  /// them due to a power failure.  The STATUS register is not verified because
  /// it does not contain any driver settings.
  ///
  /// @return 1 if the settings from the device match the cached copies, 0 if
  /// they do not.
  uint16_t verifySettings(StepperDRVConfig s, uint16_t M)
  {
    // Bit 10 in TORQUE is write-only and will always read as 0.
    return readReg((HPSDRegAddr)CTRL, M)   == s.ctrl   &&
           readReg((HPSDRegAddr)TORQUE, M) == (s.torque & ~(1 << 10)) &&
           readReg((HPSDRegAddr)OFF, M)    == s.off    &&
           readReg((HPSDRegAddr)BLANK, M)  == s.blank  &&
           readReg((HPSDRegAddr)DECAY, M)  == s.decay  &&
           readReg((HPSDRegAddr)STALL, M)  == s.stall  &&
           readReg((HPSDRegAddr)DRIVE, M)  == s.drive;
  }



  /// Enables the driver (ENBL = 1).
  void enableDriver(StepperDRVConfig *s)
  {
    s->ctrl |= (1 << 0);
    writeCTRL(*s);
  }

  /// Disables the driver (ENBL = 0).
  void disableDriver(StepperDRVConfig *s)
  {
    s->ctrl &= ~(1 << 0);
    writeCTRL(*s);
  }

  /// Sets the motor direction (RDIR).
  ///
  /// Allowed values are 0 or 1.
  ///
  /// You can use this command to control the direction of the stepper motor and
  /// leave the DIR pin disconnected.
  void setDirection(StepperDRVConfig *s, uint8_t value)
  {
    if (value)
    {
      s->ctrl |= (1 << 1);
    }
    else
    {
      s->ctrl &= ~(1 << 1);
    }
    writeCTRL(*s);
  }

  /// Returns the cached value of the motor direction (RDIR).
  ///
  /// This does not perform any SPI communication with the driver.
  uint8_t getDirection(StepperDRVConfig *s)
  {
    return s->ctrl >> 1 & 1;
  }

  /// Advances the indexer by one step (RSTEP = 1).
  ///
  /// You can use this command to step the stepper motor and leave the STEP pin
  /// disconnected.
  ///
  /// The driver automatically clears the RSTEP bit after it is written.
  void step(StepperDRVConfig s)
  {
		HPSDRegAddr RegAddr=(HPSDRegAddr)CTRL;
    writeReg(RegAddr, (s.ctrl | (1 << 2)) , s.M );
  }

  /// Sets the driver's stepping mode (MODE).
  ///
  /// This affects many things about the performance of the motor, including how
  /// much the output moves for each step taken and how much current flows
  /// through the coils in each stepping position.
  ///
  /// If an invalid stepping mode is passed to this function, then it selects
  /// 1/4 micro-step, which is the driver's default.
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// sd.setStepMode(HPSDStepMode::MicroStep32);
  /// ~~~
  void setStepMode(StepperDRVConfig *s,HPSDStepMode mode)
  {
    // Pick 1/4 micro-step by default.
    uint8_t sm = 0x03;

    switch (mode)
    {
    case MicroStep1:   sm = 0x0; break;
    case MicroStep2:   sm = 0x01; break;
    case MicroStep4:   sm = 0x02; break;
    case MicroStep8:   sm = 0x03; break;
    case MicroStep16:  sm = 0x04; break;
    case MicroStep32:  sm = 0x05; break;
    case MicroStep64:  sm = 0x06; break;
    case MicroStep128: sm = 0x07; break;
    case MicroStep256: sm = 0x08; break;
    }

    s->ctrl = (s->ctrl & 0x0F87) | (sm << 3);
    writeCTRL(*s);
  }

  /// Sets the driver's stepping mode (MODE).
  ///
  /// This version of the function allows you to express the requested
  /// microstepping ratio as a number directly.
  ///
  /// ~~~{.cpp}
  /// sd.setStepMode(32);
  /// ~~~
  ///

  /// Sets the current limit for a High-Power Stepper Motor Driver 36v4.
  ///
  /// The argument to this function should be the desired current limit in
  /// milliamps.
  ///
  /// WARNING: The 36v4 can supply up to about 4 A per coil continuously;
  /// higher currents might be sustainable for short periods, but can eventually
  /// cause the MOSFETs to overheat, which could damage them.  See the driver's
  /// product page for more information.
  ///
  /// This function allows you to set a current limit of up to 8 A (8000 mA),
  /// but we strongly recommend against using a current limit higher than 4 A
  /// (4000 mA) unless you are careful to monitor the MOSFETs' temperatures
  /// and/or restrict how long the driver uses the higher current limit.
  ///
  /// This function takes care of setting appropriate values for ISGAIN and
  /// TORQUE to get the desired current limit.
  void setCurrentMilliamps36v4(StepperDRVConfig *s,uint16_t current)
  {
    if (current > 8000) { current = 8000; }

    // From the DRV8711 datasheet, section 7.3.4, equation 2:
    //
    //   Ifs = (2.75 V * TORQUE) / (256 * ISGAIN * Risense)
    //
    // Rearranged:
    //
    //   TORQUE = (256 * ISGAIN * Risense * Ifs) / 2.75 V
    //
    // The 36v4 has an Risense of 30 milliohms, and "current" is in milliamps,
    // so:
    //
    //   TORQUE = (256 * ISGAIN * (30/1000) ohms * (current/1000) A) / 2.75 V
    //          = (7680 * ISGAIN * current) / 2750000
    //
    // We want to pick the highest gain (5, 10, 20, or 40) that will not
    // overflow TORQUE (8 bits, 0xFF max), so we start with a gain of 40 and
    // calculate the TORQUE value needed.
    uint8_t isgainBits = 0x03;
    uint16_t torqueBits = ((uint32_t)768  * current) / 6875;

    // Halve the gain and TORQUE until the TORQUE value fits in 8 bits.
    while (torqueBits > 0xFF)
    {
      isgainBits--;
      torqueBits >>= 1;
    }

    s->ctrl = (s->ctrl & 0x0CFF) | (isgainBits << 8);
    writeCTRL(*s);
    s->torque = (s->torque & 0x0F00) | torqueBits;
    writeTORQUE(*s);
  }

  /// Sets the driver's decay mode (DECMOD).
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// sd.setDecayMode(HPSDDecayMode::AutoMixed);
  /// ~~~
  void setDecayMode(StepperDRVConfig *s,HPSDDecayMode mode)
  {
    s->decay = (s->decay & 0x00FF) | (((uint8_t)mode & 0x07) << 8);
    writeDECAY(*s);
  }

  /// Reads the status of the driver (STATUS register).
  ///
  /// The return value is an 8-bit unsigned integer that has one bit for each
  /// status condition (the upper 4 bits of the 12-bit STATUS register are not
  /// used).  You can simply compare the return value to 0 to see if any of the
  /// status bits are set, or you can use the logical AND operator (`&`) and the
  /// #HPSDStatusBit enum to check individual bits.
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// if (sd.readStatus() & (1 << (uint8_t)HPSDStatusBit::UVLO))
  /// {
  ///   // Undervoltage lockout is active.
  /// }
  /// ~~~
  uint8_t readStatus(uint16_t M)
  {
    return SPIreadReg((HPSDRegAddr)STATUS, M);
  }

  /// Clears any status conditions that are currently latched in the driver.
  ///
  /// WARNING: Calling this function clears latched faults, which might allow
  /// the motor driver outputs to reactivate.  If you do this repeatedly without
  /// fixing an abnormal condition (like a short circuit), you might damage the
  /// driver.
  void clearStatus(StepperDRVConfig s)
  {
    SPIwriteReg((HPSDRegAddr)STATUS, 0, s.M);
  }

  /// Reads fault conditions indicated by the driver.
  ///
  /// The return value is the same as that which would be returned by
  /// readStatus(), except it only contains bits that indicate faults (STATUS
  /// bits 5:0).
  uint8_t readFaults(uint16_t M)
  {
    return readStatus(M) & 0x003F;
  }

  /// Clears any fault conditions that are currently latched in the driver.
  ///
  /// This function behaves the same as clearStatus(), except it only clears
  /// bits that indicate faults (STATUS bits 5:0).
  ///
  /// WARNING: Calling this function clears latched faults, which might allow
  /// the motor driver outputs to reactivate.  If you do this repeatedly without
  /// fixing an abnormal condition (like a short circuit), you might damage the
  /// driver.
  void clearFaults(StepperDRVConfig s)
  {
    writeReg((HPSDRegAddr)STATUS, ~0x003F , s.M);
  }

  /// Writes the cached value of the CTRL register to the device.
  void writeCTRL(StepperDRVConfig s)
  {
    writeReg((HPSDRegAddr)CTRL, s.ctrl, s.M);
  }

  /// Writes the cached value of the TORQUE register to the device.
  void writeTORQUE(StepperDRVConfig s)
  {
    writeReg((HPSDRegAddr)TORQUE, s.torque, s.M);
  }

  /// Writes the cached value of the OFF register to the device.
  void writeOFF(StepperDRVConfig s)
  {
    writeReg((HPSDRegAddr)OFF, s.off , s.M);
  }

  /// Writes the cached value of the BLANK register to the device.
  void writeBLANK(StepperDRVConfig s)
  {
    writeReg((HPSDRegAddr)BLANK, s.blank, s.M);
  }

  /// Writes the cached value of the DECAY register to the device.
  void writeDECAY(StepperDRVConfig s)
  {
    writeReg((HPSDRegAddr)DECAY, s.decay , s.M);
  }

  /// Writes the cached value of the STALL register to the device.
  void writeSTALL(StepperDRVConfig s)
  {
    writeReg((HPSDRegAddr)STALL, s.stall, s.M);
  }

  /// Writes the cached value of the DRIVE register to the device.
  void writeDRIVE(StepperDRVConfig s)
  {
    writeReg((HPSDRegAddr)DRIVE, s.drive, s.M);
  }

void packDataTX(uint8_t *pld, runningParams r, payloadData p){
	uint8_t payload[8];
	memset(payload,0,sizeof(payload));
	uint16_t tempTV;
	float tvFactor=0;
	
	payload[0]=(uint8_t)(key0 & key0Mask);
	payload[1]=(uint8_t)(key1 & key0Mask);
	payload[6]=(uint8_t)(key2 & key2Mask);
	payload[7]=(uint8_t)(key3 & key3Mask);
	
	//tvFactor=0;
	tvFactor=r.TV;
	if(tvFactor>750) tvFactor=600;   //max sensor range 200-550 devide it into 255 value
	//if(tvFactor<=0)tvFactor=200;
	//tvFactor-=200;
	tvFactor/=2.8; //old  tvFactor/=2.8;
	
	tempTV=(uint16_t)tvFactor;
	payload[TVindex]|=(uint8_t)((tempTV << TVOffset) & TVMask);
	payload[IEindex]|=(uint8_t)((r.IE << IEOffset) & IEMask);
	
	payload[RRindex]|=(uint8_t)((r.RR << RROffset) & RRMask);  // befor tempTV

	payload[subStindex]|=(uint8_t)((p.subState << subStOffset)& subStMask) ;
	payload[stateindex]|=(uint8_t)((p.state << stateOffset) & stateMask) ;

	payload[CMDindex]|=(uint8_t)((p.mCMD << CMDOffset) & CMDMask) ;
	payload[FLTindex]|=(uint8_t)((p.fault << FLTOffset) & FLTMask) ;	
	
	memcpy(pld,payload,sizeof(payload));
	
}

void unpackDataRX(uint8_t payload[], runningParams *r, payloadData *p){
	
	uint16_t tempTV;

	/*payload[0]=(uint8_t)(key0 & key0Mask);
	payload[1]=(uint8_t)(key1 & key0Mask);
	payload[6]=(uint8_t)(key2 & key2Mask);
	payload[7]=(uint8_t)(key3 & key3Mask);*/
	
	tempTV =	(payload[TVindex]& TVMask)>>TVOffset;
	r->TV = tempTV*2.8;   //old r->TV = tempTV*2.35;
	//r->TV = (tempTV * 50) +200u;
	r->IE =	(payload[IEindex]& IEMask)>>IEOffset;
	
	r->RR =	(payload[RRindex]& RRMask)>>RROffset;
	
	p->subState =	(payload[subStindex]& subStMask)>>subStOffset;
	p->state =	(payload[stateindex]& stateMask)>>stateOffset;
	
	p->fault =	(payload[FLTindex]& FLTMask)>>FLTOffset;
	p->mCMD =	(payload[CMDindex]& CMDMask)>>CMDOffset;
		
		/*if(p->mCMD == 0x02)
		{
			//this is two line used for motor driver come out of the sleep mode.
			HAL_GPIO_WritePin(SLEEP_M1_GPIO_Port,SLEEP_M1_Pin,GPIO_PIN_SET); //SET=Reverse
			HAL_GPIO_WritePin(SLEEP_M2_GPIO_Port,SLEEP_M2_Pin,GPIO_PIN_SET); //SET=Reverse
		}*/

}

void peep_pip_unpackDataRX(uint8_t peep_pip_payload[], uint8_t rec)
{
	rec =	(peep_pip_payload[PIPindex]& PIPMask)>>PIPOffset;
//	r->PIP =	(peep_pip_payload[PEEPindex]& PEEPMask)>>PEEPOffset;
	
}


