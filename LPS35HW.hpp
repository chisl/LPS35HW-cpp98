/*
 * name:        LPS35HW
 * description: 260-1260 hPa absolute digital output barometer with water resistant package
 * manuf:       STMicroelectronics
 * version:     Version 0.1
 * url:         http://www.st.com/resource/en/datasheet/lps35hw.pdf
 * date:        2018-01-04
 * author       https://chisl.io/
 * file:        LPS35HW.hpp
 */

/*                                                                                                       *
 *                                   THIS FILE IS AUTOMATICALLY CREATED                                  *
 *                                    D O     N O T     M O D I F Y  !                                   *
 *                                                                                                       */

#include <cinttypes>

/* Derive from class LPS35HW_Base and implement the read and write functions! */

/* LPS35HW: 260-1260 hPa absolute digital output barometer with water resistant package */
class LPS35HW_Base
{
public:
	/* Pure virtual functions that need to be implemented in derived class: */
	virtual uint8_t read8(uint16_t address, uint16_t n=8) = 0;  // 8 bit read
	virtual void write(uint16_t address, uint8_t value, uint16_t n=8) = 0;  // 8 bit write
	virtual uint16_t read16(uint16_t address, uint16_t n=16) = 0;  // 16 bit read
	virtual void write(uint16_t address, uint16_t value, uint16_t n=16) = 0;  // 16 bit write
	virtual uint32_t read32(uint16_t address, uint16_t n=32) = 0;  // 32 bit read
	virtual void write(uint16_t address, uint32_t value, uint16_t n=32) = 0;  // 32 bit write
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                        REG INTERRUPT_CFG                                         *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG INTERRUPT_CFG:
	 * 8.1
	 * To generate an interrupt event based on a user defined threshold, DIFF_EN bit must be set
	 * to '1' and the threshold values stored in THS_P_L (0Ch) and THS_P_H (0Dh).
	 * When DIFF_EN = '1', PHE bit or PLE bit or both bits have to be enabled. PHE and PLE bits
	 * enable the interrupt generation on the positive or negative event respectively.
	 * When DIFF_EN is enabled and AUTOZERO or AUTORIFP is enabled, the defined
	 * pressure threshold values in THS_P (0Ch, 0Dh) is compared with:
	 * P_DIFF_IN=measured pressure - pressure reference
	 * The value of pressure reference is assigned depending on the AUTOZERO and
	 * AUTORIFP modes reported in the next two paragraphs.
	 * 
	 * If AUTOZERO bit is set to '1', the measured pressure is used as reference on the register
	 * REF_P (15h, 16h and 17h). From now on, the output pressure registers PRESS_OUT
	 * (PRESS_OUT_H(2Ah), PRESS_OUT_L(29h) and PRESS_OUT_XL(28h)) are updated
	 * and the same value is also used for the interrupt generation: PRESS_OUT = measured
	 * pressure - REF_P
	 * After the first conversion AUTOZERO bit is automatically set to '0'. To return back to
	 * normal mode, RESET_AZ bit has to be set to '1'. This reset also the content of the REF_P
	 * registers. If AUTORIFP bit is set to '1', the measured pressure is used as reference on the
	 * register REF_P (15h, 16h and 17h). The output registers PRESS_OUT
	 * (PRESS_OUT_H(2Ah), PRESS_OUT_L(29h) and PRESS_OUT_XL(28h)) show the
	 * difference between the measured pressure and the content of the RPDS registers (18h and
	 * 19h): PRESS_OUT = measured pressure - RPDS*256
	 * After the first conversion AUTORIFP bit is automatically set to '0'. To return back to normal
	 * mode, RESET_ARP bit has to be set to '1'.
	 */
	struct INTERRUPT_CFG
	{
		static const uint16_t __address = 11;
		
		/* Bits AUTORIFP: */
		/* AutoRifP function enable.  */
		struct AUTORIFP
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t NORMAL_MODE = 0b0; // normal mode
			static const uint8_t AUTORIFP_ENABLED = 0b1; // AutoRifP enabled
		};
		/* Bits RESET_ARP: */
		/* Reset AutoRifP function.  */
		struct RESET_ARP
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b01000000; // [6]
			static const uint8_t NORMAL_MODE = 0b0; // normal mode
			static const uint8_t AUTORIFP_RESET = 0b1; // reset AutoRifP function
		};
		/* Bits AUTOZERO: */
		/* Autozero enable.  */
		struct AUTOZERO
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00100000; // [5]
			static const uint8_t NORMAL_MODE = 0b0; // normal mode
			static const uint8_t ENABLE_AUTOZERO = 0b1; // Autozero enabled
		};
		/* Bits RESET_AZ: */
		/* Reset Autozero function.  */
		struct RESET_AZ
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00010000; // [4]
			static const uint8_t NORMAL_MODE = 0b0; // normal mode
			static const uint8_t RESET_AUTOZERO = 0b1; // reset Autozero function
		};
		/* Bits DIFF_EN: */
		/* Interrupt generation enable.  */
		struct DIFF_EN
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
			static const uint8_t DISABLED = 0b0; // interrupt generation disabled
			static const uint8_t ENABLED = 0b1; // interrupt generation enabled)
		};
		/* Bits LIR: */
		/* Latch interrupt request to the INT_SOURCE register.  */
		struct LIR
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t NOT_LATCHED = 0b0; // interrupt request not latched
			static const uint8_t LATHED = 0b1; // interrupt request latched
		};
		/* Bits PLE: */
		/* Enable interrupt generation on differential pressure low event.  */
		struct PLE
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t DISABLE = 0b0; // disable interrupt request 
			static const uint8_t ENABLE = 0b1; // enable interrupt request on measured differential pressure value lower than preset threshold
		};
		/* Bits PHE: */
		/* Enable interrupt generation on differential pressure high event.  */
		struct PHE
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t DISABLE = 0b0; // disable interrupt request
			static const uint8_t ENABLE = 0b1; // enable interrupt request on measured differential pressure value higher than preset threshold
		};
	};
	
	/* Set register INTERRUPT_CFG */
	void setINTERRUPT_CFG(uint8_t value)
	{
		write(INTERRUPT_CFG::__address, value, 8);
	}
	
	/* Get register INTERRUPT_CFG */
	uint8_t getINTERRUPT_CFG()
	{
		return read8(INTERRUPT_CFG::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                            REG THS_P                                             *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG THS_P:
	 * 8.2-3
	 * Threshold value for pressure interrupt generation.
	 * This register contains the threshold value for pressure interrupt generation.
	 */
	struct THS_P
	{
		static const uint16_t __address = 12;
		
		/* Bits THS: */
		/* Refer to Section 10.2: "THS_P_L (0Ch)"  */
		struct THS
		{
			static const uint16_t mask = 0b1111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
		};
	};
	
	/* Set register THS_P */
	void setTHS_P(uint16_t value)
	{
		write(THS_P::__address, value, 16);
	}
	
	/* Get register THS_P */
	uint16_t getTHS_P()
	{
		return read16(THS_P::__address, 16);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG WHO_AM_I                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG WHO_AM_I:
	 * 8.4
	 * Device Who am I
	 */
	struct WHO_AM_I
	{
		static const uint16_t __address = 15;
		
		/* Bits WHO_AM_I: */
		struct WHO_AM_I_
		{
			/* MODE - */
			static const uint8_t dflt = 0b10110001; // 8'b10110001
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register WHO_AM_I */
	void setWHO_AM_I(uint8_t value)
	{
		write(WHO_AM_I::__address, value, 8);
	}
	
	/* Get register WHO_AM_I */
	uint8_t getWHO_AM_I()
	{
		return read8(WHO_AM_I::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG CTRL_REG1                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG CTRL_REG1:
	 * 8.5
	 * Control register 1
	 * When ODR bits are set to '000' the device is in Power down mode. When the device is in
	 * power-down mode, almost all internal blocks of the device are switched off to minimize
	 * power consumption. I2C interface is still active to allow communication with the device. The
	 * configuration registers content is preserved and output data registers are not updated,
	 * therefore keeping the last data sampled in memory before going into power- down mode.
	 * If ONE_SHOT bit in CTRL_REG2(11h) is set to '1', One-shot mode is triggered and a new
	 * acquisition starts when it is required. This enabling is effective only if the device was
	 * previously in power-down mode (ODR bits set to '000'). Once the acquisition is completed
	 * and the output registers updated, the device automatically enters in power down mode.
	 * ONE_SHOT bit self-clears itself.
	 * When ODR bits are set to a value different than '000', the device is in Continuous mode
	 * and automatically acquires a set of data (pressure and temperature) at the frequency
	 * selected through ODR[2,0] bits.
	 * 
	 * Once the additional low pass filter has been enable through the EN_LPFP bit, it is possible
	 * to configure the device bandwidth acting on LPFP_CFG bit. Refer to Table 20: "Low-pass
	 * filter configurations".
	 * 
	 * Table 20: Low-pass filter configurations
	 * EN_LPFP LPFP_CFG Additional low pass filter status    Device bandwidth
	 * 0       x        Disabled                             ODR/2
	 * 1       0        Enabled                              ODR/9
	 * 1       1        Enabled                              ODR/20
	 * 
	 * The BDU bit is used to inhibit the update of the output registers between the reading of
	 * upper and lower register parts. In default mode (BDU = ‘0’), the lower and upper register
	 * parts are updated continuously. When the BDU is activated (BDU = ‘1’), the content of the
	 * output registers is not updated until PRESS_OUT_H is read, avoiding the reading of values
	 * related to different samples.
	 */
	struct CTRL_REG1
	{
		static const uint16_t __address = 16;
		
		/* Bits unused_0: */
		/* This bit must be set to ‘0’ for proper operation of the device  */
		struct unused_0
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits ODR: */
		/* Output data rate selection.  */
		struct ODR
		{
			static const uint8_t dflt = 0b000; // 3'b0
			static const uint8_t mask = 0b01110000; // [4,5,6]
			static const uint8_t POWER_DOWN_ONE_SHOT = 0b00; // Power down / One shot mode enabled
			static const uint8_t F_1_HZ = 0b01; // p:  1 Hz, T:  1 Hz
			static const uint8_t F_10_HZ = 0b10; // p: 10 Hz, T: 10 Hz
			static const uint8_t F_25_HZ = 0b11; // p: 25 Hz, T: 25 Hz
			static const uint8_t F_50_HZ = 0b100; // p: 50 Hz, T: 50 Hz
			static const uint8_t F_75_HZ = 0b101; // p: 75 Hz, T: 75 Hz
		};
		/* Bits EN_LPFP: */
		/* Enable low-pass filter on pressure data.  */
		struct EN_LPFP
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
			static const uint8_t DISABLE = 0b0; // Low-pass filter disabled
			static const uint8_t ENABLE = 0b1; // Low-pass filter enabled
		};
		/* Bits LPFP_CFG: */
		/*
		 * Low-pass configuration register.
		 * Refer to Table 20: "Low- pass filter configurations".
		 */
		struct LPFP_CFG
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
		};
		/* Bits BDU: */
		/*
		 * Block data update.
		 * To guarantee the correct behavior of BDU feature, th PRESS_OUT_H (2Ah) must be the last address read.
		 */
		struct BDU
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t CONTINUOUS_UPDATE = 0b0; // continuous update;
			static const uint8_t NOT_UPDATED_UNTIL_READ = 0b1; // output registers not updated until MSB and LSB have been read
		};
		/* Bits SIM: */
		/* SPI Serial Interface Mode selection.  */
		struct SIM
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t SPI_4_WIRE = 0b0; // 4-wire interface
			static const uint8_t SPI_3_WIRE = 0b1; // 3-wire interface
		};
	};
	
	/* Set register CTRL_REG1 */
	void setCTRL_REG1(uint8_t value)
	{
		write(CTRL_REG1::__address, value, 8);
	}
	
	/* Get register CTRL_REG1 */
	uint8_t getCTRL_REG1()
	{
		return read8(CTRL_REG1::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG CTRL_REG2                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG CTRL_REG2:
	 * 8.6
	 * Control register 2
	 */
	struct CTRL_REG2
	{
		static const uint16_t __address = 17;
		
		/* Bits BOOT: */
		/*
		 * Reboot memory content.
		 * The bit is self-cleared when the BOOT is completed.
		 * The BOOT bit is used to refresh the content of the internal registers stored in the Flash
		 * memory block. At device power-up the content of the Flash memory block is transferred to
		 * the internal registers related to the trimming functions to allow correct behavior of the
		 * device itself. If for any reason the content of the trimming registers is modified, it is
		 * sufficient to use this bit to restore the correct values. When the BOOT bit is set to ‘1’, the
		 * content of the internal Flash is copied inside the corresponding internal registers and is
		 * used to calibrate the device. These values are factory trimmed and they are different for
		 * every device. They allow correct behavior of the device and normally they should not be
		 * changed. At the end of the boot process the BOOT bit is set again to ‘0’ by hardware. The
		 * BOOT bit takes effect after one ODR clock cycle.
		 */
		struct BOOT
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t NORMAL_MODE = 0b0; // normal mode
			static const uint8_t REBOOT_MEMORY = 0b1; // reboot memory content
		};
		/* Bits FIFO_EN: */
		/* FIFO enable.  */
		struct FIFO_EN
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b01000000; // [6]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits STOP_ON_FTH: */
		/* Stop on FIFO threshold. Enable FIFO watermark level use.  */
		struct STOP_ON_FTH
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00100000; // [5]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits IF_ADD_INC: */
		/*
		 * Register address automatically incremented during a multiple byte access with a
		 * serial interface (I2C or SPI).
		 */
		struct IF_ADD_INC
		{
			static const uint8_t dflt = 0b1; // 1'b1
			static const uint8_t mask = 0b00010000; // [4]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits I2C_DIS: */
		/* Disable I2C interface.  */
		struct I2C_DIS
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
			static const uint8_t ENABLE = 0b0; // I2C enabled
			static const uint8_t DISABLE = 0b1; // I2C disabled
		};
		/* Bits SWRESET: */
		/*
		 * Software reset.
		 * The bit is self-cleared when the reset is completed.
		 * SWRESET is the software reset bit. The following device registers (INTERRUPT_CFG,
		 * THS_P_L, THS_P_H, CTRL_REG1, CTRL_REG2, CTRL_REG3, FIFO_CTRL,
		 * RIF_P_XL,RIF_P_L,RIF_P_H) are reset to the default value if the SWRESET bit is set to
		 * '1'. SWRESET bit comes back to '0' by hardware.
		 */
		struct SWRESET
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t NORMAL_MODE = 0b0; // normal mode
			static const uint8_t SOFTWARE_RESET = 0b1; // software reset
		};
		/* Bits unused_0: */
		/* This bit must be set to ‘0’ for proper operation of the device.  */
		struct unused_0
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
		};
		/* Bits ONE_SHOT: */
		/*
		 * One-shot enable.
		 * The ONE_SHOT bit is used to start a new conversion when the ODR[2,0] bits in
		 * CTRL_REG1(10h)are set to ‘000’. Writing a ‘1’ in ONE_SHOT triggers a single
		 * measurement of pressure and temperature. Once the measurement is done, the
		 * ONE_SHOT bit will self-clear, the new data are available in the output registers, and the
		 * STATUS_REG bits are updated.
		 */
		struct ONE_SHOT
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t IDLE_MODE = 0b0; // idle mode
			static const uint8_t NEW_DATASET = 0b1; // a new dataset is acquired
		};
	};
	
	/* Set register CTRL_REG2 */
	void setCTRL_REG2(uint8_t value)
	{
		write(CTRL_REG2::__address, value, 8);
	}
	
	/* Get register CTRL_REG2 */
	uint8_t getCTRL_REG2()
	{
		return read8(CTRL_REG2::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG CTRL_REG3                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG CTRL_REG3:
	 * 8.7
	 * Control register 3 - INT_DRDY pin control register
	 */
	struct CTRL_REG3
	{
		static const uint16_t __address = 18;
		
		/* Bits INT_H_L: */
		/* Interrupt active-high/low.  */
		struct INT_H_L
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t ACTIVE_HIGH = 0b0; // active high
			static const uint8_t ACTIVE_LOW = 0b1; // active low
		};
		/* Bits PP_OD: */
		/* Push-pull/open drain selection on interrupt pads.  */
		struct PP_OD
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b01000000; // [6]
			static const uint8_t PUSH_PULL = 0b0; // push-pull
			static const uint8_t OPEN_DRAIN = 0b1; // open drain
		};
		/* Bits F_FSS5: */
		/* FIFO full flag on INT_DRDY pin.  */
		struct F_FSS5
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00100000; // [5]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits F_FTH: */
		/* FIFO threshold (Watermark) status on INT_DRDY pin.  */
		struct F_FTH
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00010000; // [4]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits F_OVR: */
		/* FIFO overrun interrupt on INT_DRDY pin.  */
		struct F_OVR
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits DRDY: */
		/* Data-ready signal on INT_DRDY pin.  */
		struct DRDY
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits INT_S: */
		/* Data signal on INT_DRDY pin control bits.  */
		struct INT_S
		{
			static const uint8_t dflt = 0b00; // 2'b0
			static const uint8_t mask = 0b00000011; // [0,1]
			static const uint8_t DATA_SIGNAL = 0b00; // Data signal (in order of priority: PTH_DRDY or F_FTH or F_OVR or F_FSSS5)
			static const uint8_t PRESSURE_HIGH = 0b01; // Pressure high (P_high)
			static const uint8_t PRESSURE_LOW = 0b10; // Pressure low (P_low)
			static const uint8_t PRESSURE_LOW_OR_HIGH = 0b11; // Pressure low OR high
		};
	};
	
	/* Set register CTRL_REG3 */
	void setCTRL_REG3(uint8_t value)
	{
		write(CTRL_REG3::__address, value, 8);
	}
	
	/* Get register CTRL_REG3 */
	uint8_t getCTRL_REG3()
	{
		return read8(CTRL_REG3::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG FIFO_CTRL                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG FIFO_CTRL:
	 * 8.8
	 * FIFO control register
	 */
	struct FIFO_CTRL
	{
		static const uint16_t __address = 20;
		
		/* Bits F_MODE: */
		/* FIFO mode selection.  */
		struct F_MODE
		{
			static const uint8_t dflt = 0b000; // 3'b0
			static const uint8_t mask = 0b11100000; // [5,6,7]
			static const uint8_t BYPASS = 0b00; // Bypass mode
			static const uint8_t FIFO = 0b01; // FIFO mode
			static const uint8_t STREAM = 0b10; // Stream mode
			static const uint8_t STREAM_TO_FIFO = 0b11; // Stream-to-FIFO mode
			static const uint8_t BYPASS_TO_STREAM = 0b100; // Bypass-to-Stream mode
			static const uint8_t reserved_0 = 0b101; // Reserved
			static const uint8_t DYNAMIC_STREAM = 0b110; // Dynamic-Stream mode
			static const uint8_t BYPASS_TO_FIFO = 0b111; // Bypass-to-FIFO mode
		};
		/* Bits WTM: */
		/* FIFO watermark level selection.  */
		struct WTM
		{
			static const uint8_t mask = 0b00011111; // [0,1,2,3,4]
		};
	};
	
	/* Set register FIFO_CTRL */
	void setFIFO_CTRL(uint8_t value)
	{
		write(FIFO_CTRL::__address, value, 8);
	}
	
	/* Get register FIFO_CTRL */
	uint8_t getFIFO_CTRL()
	{
		return read8(FIFO_CTRL::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                            REG REF_P                                             *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG REF_P:
	 * 8.9
	 * Reference pressure
	 * The value is expressed as 2’s complement.
	 * The reference pressure value is used when AUTOZERO or AUTORIFP function is
	 * enabled(refer to the Section 10.7: "CTRL_REG3 (12h)" register) and for the Autozero
	 * function (refer to the Section 10.1: "INTERRUPT_CFG (0Bh)" register).
	 */
	struct REF_P
	{
		static const uint16_t __address = 21;
		
		/* Bits REFL: */
		/*
		 * This register contains the low part of the reference pressure value.
		 * The Reference pressure value is a 24-bit data and it is composed of Section 10.11:
		 * "REF_P_H_17h", Section 10.10: "REF_P_L_16h" and Section 10.9: "REF_P_XL (15h)".
		 */
		struct REFL
		{
			static const uint32_t mask = 0b111111111111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]
		};
	};
	
	/* Set register REF_P */
	void setREF_P(uint32_t value)
	{
		write(REF_P::__address, value, 24);
	}
	
	/* Get register REF_P */
	uint32_t getREF_P()
	{
		return read32(REF_P::__address, 24);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                             REG RPDS                                              *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG RPDS:
	 * 8.12
	 * Pressure offset
	 * If, after the soldering of the component, a residual offset is still present, it can be removed
	 * with a one-point calibration.
	 * After the soldering, the measured offset can be stored in the Section 10.13:
	 * "RPDS_H_19h" and Section 10.12: "RPDS_L_18h" registers and automatically subtracted
	 * from the pressure output registers: the output pressure register PRESS_OUT (28h, 29h
	 * and 2Ah) is provided as the difference between the measured pressure and the content of
	 * the register 256*RPDS (18h, 19h)*.
	 * *DIFF_EN = '0', AUTOZERO = '0', AUTORIFP = '0'
	 */
	struct RPDS
	{
		static const uint16_t __address = 24;
		
		/* Bits RPDS: */
		struct RPDS_
		{
			/* MODE - */
			static const uint16_t mask = 0b1111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
		};
	};
	
	/* Set register RPDS */
	void setRPDS(uint16_t value)
	{
		write(RPDS::__address, value, 16);
	}
	
	/* Get register RPDS */
	uint16_t getRPDS()
	{
		return read16(RPDS::__address, 16);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG RES_CONF_1                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG RES_CONF_1:
	 * 8.14
	 * Low-power mode configuration
	 */
	struct RES_CONF_1
	{
		static const uint16_t __address = 26;
		
		/* Bits unused_0: */
		/* These bits must be set to ‘0’ for proper operation of the device.  */
		struct unused_0
		{
			static const uint8_t dflt = 0b000000; // 6'd0
			static const uint8_t mask = 0b11111100; // [2,3,4,5,6,7]
		};
		/* Bits reserved_1: */
		/* The content of this bit must not be modified for proper operation of the device  */
		struct reserved_1
		{
			static const uint8_t mask = 0b00000010; // [1]
		};
		/* Bits LC_EN: */
		/*
		 * Low current mode enable.
		 * The LC_EN bit must be changed only with the device in power down and not during operation. Once LC_EN bit
		 * is configured, it affects both One-shot mode and Continuous mode.
		 */
		struct LC_EN
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t NORMAL_MODE = 0b0; // Normal mode (low-noise mode)
			static const uint8_t LOW_CURRENT_MODE = 0b1; // Low-current mode
		};
	};
	
	/* Set register RES_CONF_1 */
	void setRES_CONF_1(uint8_t value)
	{
		write(RES_CONF_1::__address, value, 8);
	}
	
	/* Get register RES_CONF_1 */
	uint8_t getRES_CONF_1()
	{
		return read8(RES_CONF_1::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG INT_SOURCE                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG INT_SOURCE:
	 * 8.15
	 * Interrupt source
	 */
	struct INT_SOURCE
	{
		static const uint16_t __address = 37;
		
		/* Bits BOOT_STATUS: */
		/* If ‘1’ indicates that the Boot (Reboot) phase is running.  */
		struct BOOT_STATUS
		{
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits unused_0: */
		/* These bits must be set to ‘0’ for proper operation of the device.  */
		struct unused_0
		{
			static const uint8_t dflt = 0b0000; // 4'd0
			static const uint8_t mask = 0b01111000; // [3,4,5,6]
		};
		/* Bits IA: */
		/* Interrupt active.  */
		struct IA
		{
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t NO_INTERRUPT = 0b0; // no interrupt has been generated;
			static const uint8_t EVENT = 0b1; // one or more interrupt events have been generated
		};
		/* Bits PL: */
		/* Differential pressure Low.  */
		struct PL
		{
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t NO_INTERRUPT = 0b0; // no interrupt has been generated;
			static const uint8_t EVENT = 0b1; // Low differential pressure event has occurred
		};
		/* Bits PH: */
		/* Differential pressure High.  */
		struct PH
		{
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t NO_INTERRUPT = 0b0; // no interrupt has been generated;
			static const uint8_t EVENT = 0b1; // High differential pressure event has occurred
		};
	};
	
	/* Set register INT_SOURCE */
	void setINT_SOURCE(uint8_t value)
	{
		write(INT_SOURCE::__address, value, 8);
	}
	
	/* Get register INT_SOURCE */
	uint8_t getINT_SOURCE()
	{
		return read8(INT_SOURCE::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                         REG FIFO_STATUS                                          *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG FIFO_STATUS:
	 * 8.16
	 * FIFO status
	 * Note: When the number of unread samples in FIFO is greater than the threshold level set in
	 * register Section 10.8: "FIFO_CTRL (14h)", FTH value is ‘1’.
	 */
	struct FIFO_STATUS
	{
		static const uint16_t __address = 38;
		
		/* Bits FTH_FIFO: */
		/* FIFO threshold status.  */
		struct FTH_FIFO
		{
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t FIFO_BELOW = 0b0; // FIFO filling is lower than treshold level,
			static const uint8_t FIFO_AT_OR_ABOVE = 0b1; // FIFO filling is equal or higher than treshold level
		};
		/* Bits OVR: */
		/* FIFO overrun status.  */
		struct OVR
		{
			static const uint8_t mask = 0b01000000; // [6]
			static const uint8_t FIFO_NOT_FULL = 0b0; // FIFO is not completely full;
			static const uint8_t OVERWRITTEN = 0b1; // FIFO is full and at least one sample in the FIFO has been overwritten
		};
		/* Bits FSS: */
		/*
		 * FIFO stored data level.
		 * 6'b000000: FIFO is empty , 6'b100000: FIFO is full and has 32 unread samples.
		 * 
		 * Table 33: FIFO_STATUS example: OVR/FSS details
		 * FTH  OVRN FSS     Description
		 * 0    0    000000  FIFO empty
		 * 1    0    000001  1 unread sample
		 * ...
		 * 1    0    100000  32 unread sample
		 * 
		 */
		struct FSS
		{
			static const uint8_t mask = 0b00111111; // [0,1,2,3,4,5]
			static const uint8_t FIFO_EMPTY = 0b0000; // 
		};
	};
	
	/* Set register FIFO_STATUS */
	void setFIFO_STATUS(uint8_t value)
	{
		write(FIFO_STATUS::__address, value, 8);
	}
	
	/* Get register FIFO_STATUS */
	uint8_t getFIFO_STATUS()
	{
		return read8(FIFO_STATUS::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                            REG STATUS                                             *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG STATUS:
	 * 8.17
	 * Status register. This register is updated every ODR cycle.
	 */
	struct STATUS
	{
		static const uint16_t __address = 39;
		
		/* Bits unused_0: */
		struct unused_0
		{
			static const uint8_t mask = 0b11000000; // [6,7]
		};
		/* Bits T_OR: */
		/* Temperature data overrun.  */
		struct T_OR
		{
			static const uint8_t mask = 0b00100000; // [5]
			static const uint8_t NO_OVERRUN = 0b0; // no overrun has occurred;
			static const uint8_t OVERWRITTEN = 0b1; // a new data for temperature has overwritten the previous one
		};
		/* Bits P_OR: */
		/* Pressure data overrun.  */
		struct P_OR
		{
			static const uint8_t mask = 0b00010000; // [4]
			static const uint8_t NO_OVERRUN = 0b0; // no overrun has occurred;
			static const uint8_t OVERWRITTEN = 0b1; // new data for pressure has overwritten the previous one
		};
		/* Bits unused_1: */
		struct unused_1
		{
			static const uint8_t mask = 0b00001100; // [2,3]
		};
		/* Bits T_DA: */
		/* Temperature data available.  */
		struct T_DA
		{
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t NO_NEW_DATA = 0b0; // new data for temperature is not yet available
			static const uint8_t NEW_DATA = 0b1; // new data for temperature is available
		};
		/* Bits P_DA: */
		/* Pressure data available.  */
		struct P_DA
		{
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t NO_NEW_DATA = 0b0; // new data for pressure is not yet available;
			static const uint8_t NEW_DATA = 0b1; // new data for pressure is available
		};
	};
	
	/* Set register STATUS */
	void setSTATUS(uint8_t value)
	{
		write(STATUS::__address, value, 8);
	}
	
	/* Get register STATUS */
	uint8_t getSTATUS()
	{
		return read8(STATUS::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG PRESS_OUT                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG PRESS_OUT:
	 * 8.18-20
	 * The pressure output value is a 24-bit data that contains the measured pressure. It is
	 * composed of Section 10.20: "PRESS_OUT_H_2Ah", Section 10.19: "PRESS_OUT_L_29h"
	 * and Section 10.18: "PRESS_OUT_XL_28h" . The value is expressed as 2’s complement.
	 * The output pressure register PRESS_OUT is provided as the difference between the
	 * measured pressure and the content of the register RPDS (18h, 19h)
	 * (DIFF_EN = '0', AUTOZERO = '0', AUTORIFP = '0').
	 * Please refer to section Section 5.4: "How to interpret pressure readings" for additional info.
	 */
	struct PRESS_OUT
	{
		static const uint16_t __address = 40;
		
		/* Bits PRESS_OUT: */
		struct PRESS_OUT_
		{
			/* MODE - */
			static const uint32_t mask = 0b111111111111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]
		};
	};
	
	/* Set register PRESS_OUT */
	void setPRESS_OUT(uint32_t value)
	{
		write(PRESS_OUT::__address, value, 24);
	}
	
	/* Get register PRESS_OUT */
	uint32_t getPRESS_OUT()
	{
		return read32(PRESS_OUT::__address, 24);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG TEMP_OUT                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG TEMP_OUT:
	 * 8.21-23
	 * Temperature output value
	 * The temperature output value is a 24-bit data that contains the measured temperature. It is
	 * composed of Section 10.20: "PRESS_OUT_H_2Ah", and Section 10.18:
	 * "PRESS_OUT_XL_28h". The value is expressed as 2’s complement.
	 */
	struct TEMP_OUT
	{
		static const uint16_t __address = 43;
		
		/* Bits TEMP_OUT: */
		struct TEMP_OUT_
		{
			/* MODE - */
			static const uint32_t mask = 0b111111111111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]
		};
	};
	
	/* Set register TEMP_OUT */
	void setTEMP_OUT(uint32_t value)
	{
		write(TEMP_OUT::__address, value, 24);
	}
	
	/* Get register TEMP_OUT */
	uint32_t getTEMP_OUT()
	{
		return read32(TEMP_OUT::__address, 24);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG LPFP_RES                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG LPFP_RES:
	 * 8.23
	 * Low-pass filter reset register.
	 * If the LPFP is active, in order to avoid the transitory phase, the filter can be reset by
	 * reading this register before getting out pressure measurements.
	 */
	struct LPFP_RES
	{
		static const uint16_t __address = 51;
		
		/* Bits LPFP_RES: */
		struct LPFP_RES_
		{
			/* MODE - */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register LPFP_RES */
	void setLPFP_RES(uint8_t value)
	{
		write(LPFP_RES::__address, value, 8);
	}
	
	/* Get register LPFP_RES */
	uint8_t getLPFP_RES()
	{
		return read8(LPFP_RES::__address, 8);
	}
	
};
