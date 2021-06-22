// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/
#include "stm32f1xx_hal.h"
#include "string.h"
//#include "main.h"
extern SPI_HandleTypeDef hspi1;

#define DefaultMotorConfig {0x0C10,			  \
														0x01FF, 			\
														0x0030, 			\
														0x0080, 			\
														0x0110, 			\
														0x0040,       \
														0x0A5B,       \
														1u}
  /// The default constructor.
typedef struct StepperDRVConfigStruct{
    // All settings set to power-on defaults
    uint16_t ctrl;  
    uint16_t torque;
    uint16_t off ;   
    uint16_t blank;  
    uint16_t decay;  
    uint16_t stall;  
	  uint16_t drive; 
	  uint16_t M;
}StepperDRVConfig;
/// Addresses of control and status registers.
typedef enum HPSDRegAddrEnum
{
  CTRL   = 0x00,
  TORQUE = 0x01,
  OFF    = 0x02,
  BLANK  = 0x03,
  DECAY  = 0x04,
  STALL  = 0x05,
  DRIVE  = 0x06,
  STATUS = 0x07
	
}HPSDRegAddr;

/// Possible arguments to setStepMode().
typedef enum HPSDStepModeEnum
{
  MicroStep256 = 256u,
  MicroStep128 = 128u,
  MicroStep64  =  64u,
  MicroStep32  =  32u,
  MicroStep16  =  16u,
  MicroStep8   =   8u,
  MicroStep4   =   4u,
  MicroStep2   =   2u,
  MicroStep1   =   1u
}HPSDStepMode;

/// Possible arguments to setDecayMode().
typedef enum HPSDDecayModeEnum
{
  Slow                = 0x00,
  SlowIncMixedDec     = 0x01,
  Fast                = 0x02,
  Mixed               = 0x03,
  SlowIncAutoMixedDec = 0x04,
  AutoMixed           = 0x05
	
}HPSDDecayMode;

typedef enum HPSDStatusBitEnum
{
  /// Overtemperature shutdown
  OTS = 0,

  /// Channel A overcurrent shutdown
  AOCP = 1,

  /// Channel B overcurrent shutdown
  BOCP = 2,

  /// Channel A predriver fault
  APDF = 3,

  /// Channel B predriver fault
  BPDF = 4,

  /// Undervoltage lockout
  UVLO = 5,

  /// Stall detected
  STD = 6,

  /// Latched stall detect
  STDLAT = 7,
	
}HPSDStatusBit;


/********************************************application.h*************/



#define aniSquare1 50,80,62,92   //1
#define aniSquare4 50,96,62,108  //4
#define aniSquare2 66,80,78,92	//2
#define aniSquare3 66,96,78,108	//3				


typedef enum{
	initScreen 			=0u,
	operationScreen =1u,
	haltScreen 			=2u
}Screens;

typedef enum{
	state0					=0u,
	stateSelect			=1u,	
	stateConfig			=2u,
	stateReturn			=3u,
	stateRunning		=4u
}interactStates;

typedef enum{
	logoScreen			=0u,
	init0						=1u,
	initSensorCheck	=2u,
	connectionError	=3u,
	initDone				=4u
}initStates;

typedef enum{
	row1,
	row2,
	row3,
	row4,
	row5,
	row6
}uiRows;

typedef enum{
	noError					=0u,
	errPIP					=1u,
	errPEEP					=2u,
	errTV						=3u,
	errRR_MINvol		=4u,
	errComm         =5u,
	errMALFn				=6u,
	errSensor				=7u
}errorState;

typedef enum{
	mCMDACK=0u,
	mCMDReset=1u,
	mCMDStart=2u,
	mCMDStop=3u
}motorCommand;

typedef struct{
	uint16_t TV;
	uint16_t RR;
	uint16_t PIP;
	uint16_t PEEP;
	uint16_t MINvol;
	uint16_t IE;
}runningParams;


#define payloadMaxindex 8

#define key0index 0
#define key1index 1
#define key2index 6
#define key3index 7


#define PEEP_PIP_key0index 0
#define PEEP_PIP_key1index 1
#define PEEP_PIP_key2index 4
#define PEEP_PIP_key3index 5

#define key0 0xAA
#define key1 0x55
#define key2 0x55
#define key3 0xAA

#define PEEP_PIP_key0 0xBB
#define PEEP_PIP_key1 0x88
#define PEEP_PIP_key2 0x88
#define PEEP_PIP_key3 0xBB

#define key0Mask 0xFF
#define key1Mask 0xFF
#define key2Mask 0xFF
#define key3Mask 0xFF

#define TVOffset 0u
#define TVMask 0x00FF       //old 0x000F
#define TVindex 3u         //old 2u

#define IEOffset 5u    //old 4u
#define IEMask 	0x00E0			//old 0x00F0 //2bits max IE =3
#define IEindex 2u

#define RROffset 0u
#define RRMask 0x001F    //old ox00ff//5bits max RR=30  //before 0x1Fu //now changed it to TV 255
#define RRindex 2u

#define PIPOffset 0u
#define PIPMask 0x00FF  
#define PIPindex 2u

#define PEEPOffset 0u
#define PEEPMask 0x00FF  
#define PEEPindex 3u

#define CMDOffset 0u
#define CMDMask 0x000F
#define CMDindex 4u

#define FLTOffset 4u
#define FLTMask 0x00F0
#define FLTindex  4

#define subStOffset 0u
#define subStMask  0x000F
#define subStindex 5u

#define stateOffset 4u
#define stateMask 0x00F0
#define stateindex 5u

typedef struct{
	uint16_t state;
	uint16_t subState;
	uint16_t fault;
	uint16_t mCMD;
}payloadData;

void packDataTX(uint8_t *pld, runningParams r, payloadData p);
void unpackDataRX(uint8_t payload[], runningParams *r, payloadData *p);
void peep_pip_unpackDataRX(uint8_t peep_pip_payload[], uint8_t rec);

/***************************************************/



  /// Configures this object to use the specified pin as a chip select pin.
  ///
  /// You must use a chip select pin; the DRV8711 requires it.
  void setChipSelectPin(uint8_t pin);
	
  void selectChip1(void);

  void deselectChip1(void);

  void selectChip2(void);

  void deselectChip2(void);

	
  /// Reads the register at the given address and returns its raw value.
  uint16_t SPIreadReg(uint8_t address,uint16_t M);

  /// Reads the register at the given address and returns its raw value.
  uint16_t readReg(HPSDRegAddr address, uint16_t M);

  /// Writes the specified value to a register.
  void SPIwriteReg(uint8_t address, uint16_t value, uint16_t M);
	
  /// Writes the specified value to a register.
  void writeReg(HPSDRegAddr address, uint16_t value,uint16_t M);

  /// Re-writes the cached settings stored in this class to the device.
  ///
  /// You should not normally need to call this function because settings are
  /// written to the device whenever they are changed.  However, if
  /// verifySettings() returns false (due to a power interruption, for
  /// instance), then you could use applySettings() to get the device's settings
  /// back into the desired state.
  void applySettings(StepperDRVConfig s);


  /// Changes all of the driver's settings back to their default values.
  ///
  /// It is good to call this near the beginning of your program to ensure that
  /// there are no settings left over from an earlier time that might affect the
  /// operation of the driver.
  void resetSettings(StepperDRVConfig s);

  /// Reads back the SPI configuration registers from the device and verifies
  /// that they are equal to the cached copies stored in this class.
  ///
  /// This can be used to verify that the driver is powered on and has not lost
  /// them due to a power failure.  The STATUS register is not verified because
  /// it does not contain any driver settings.
  ///
  /// @return 1 if the settings from the device match the cached copies, 0 if
  /// they do not.
  uint16_t verifySettings(StepperDRVConfig s, uint16_t M);


  /// Enables the driver (ENBL = 1).
  void enableDriver(StepperDRVConfig *s);
	

  /// Disables the driver (ENBL = 0).
  void disableDriver(StepperDRVConfig *s);
	
  /// Sets the motor direction (RDIR).
  ///
  /// Allowed values are 0 or 1.
  ///
  /// You can use this command to control the direction of the stepper motor and
  /// leave the DIR pin disconnected.
  void setDirection(StepperDRVConfig *s, uint8_t value);
	
  /// Returns the cached value of the motor direction (RDIR).
  ///
  /// This does not perform any SPI communication with the driver.
  uint8_t getDirection(StepperDRVConfig *s);

  /// Advances the indexer by one step (RSTEP = 1).
  ///
  /// You can use this command to step the stepper motor and leave the STEP pin
  /// disconnected.
  ///
  /// The driver automatically clears the RSTEP bit after it is written.
  void step(StepperDRVConfig s);
	
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
  void setStepMode(StepperDRVConfig *s,HPSDStepMode mode);

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
  void setCurrentMilliamps36v4(StepperDRVConfig *s,uint16_t current);

  /// Sets the driver's decay mode (DECMOD).
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// sd.setDecayMode(HPSDDecayMode::AutoMixed);
  /// ~~~
  void setDecayMode(StepperDRVConfig *s,HPSDDecayMode mode);

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
  uint8_t readStatus(uint16_t M);

  /// Clears any status conditions that are currently latched in the driver.
  ///
  /// WARNING: Calling this function clears latched faults, which might allow
  /// the motor driver outputs to reactivate.  If you do this repeatedly without
  /// fixing an abnormal condition (like a short circuit), you might damage the
  /// driver.
  void clearStatus(StepperDRVConfig s);

  /// Reads fault conditions indicated by the driver.
  ///
  /// The return value is the same as that which would be returned by
  /// readStatus(), except it only contains bits that indicate faults (STATUS
  /// bits 5:0).
  uint8_t readFaults(uint16_t M);

  /// Clears any fault conditions that are currently latched in the driver.
  ///
  /// This function behaves the same as clearStatus(), except it only clears
  /// bits that indicate faults (STATUS bits 5:0).
  ///
  /// WARNING: Calling this function clears latched faults, which might allow
  /// the motor driver outputs to reactivate.  If you do this repeatedly without
  /// fixing an abnormal condition (like a short circuit), you might damage the
  /// driver.
  void clearFaults(StepperDRVConfig s);

  /// Writes the cached value of the CTRL register to the device.
  void writeCTRL(StepperDRVConfig s);

  /// Writes the cached value of the TORQUE register to the device.
  void writeTORQUE(StepperDRVConfig s);

  /// Writes the cached value of the OFF register to the device.
  void writeOFF(StepperDRVConfig s);

  /// Writes the cached value of the BLANK register to the device.
  void writeBLANK(StepperDRVConfig s);

  /// Writes the cached value of the DECAY register to the device.
  void writeDECAY(StepperDRVConfig s);
  

  /// Writes the cached value of the STALL register to the device.
  void writeSTALL(StepperDRVConfig s);

  /// Writes the cached value of the DRIVE register to the device.
  void writeDRIVE(StepperDRVConfig s);
  



