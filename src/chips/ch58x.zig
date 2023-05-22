const micro = @import("microzig");
const mmio = micro.mmio;

pub const devices = struct {
    ///  WCH wireless MCU CH58x Family Support, Drivers
    pub const CH583F = struct {
        pub const peripherals = struct {
            ///  System Control Register
            pub const SYS = @intToPtr(*volatile types.peripherals.SYS, 0x40001000);
            ///  Timer0 register
            pub const TMR0 = @intToPtr(*volatile types.peripherals.TMR0, 0x40002000);
            ///  Timer1 register
            pub const TMR1 = @intToPtr(*volatile types.peripherals.TMR1, 0x40002400);
            ///  Timer2 register
            pub const TMR2 = @intToPtr(*volatile types.peripherals.TMR2, 0x40002800);
            ///  Timer3 register
            pub const TMR3 = @intToPtr(*volatile types.peripherals.TMR3, 0x40002c00);
            ///  UART0 register
            pub const UART0 = @intToPtr(*volatile types.peripherals.UART0, 0x40003000);
            ///  UART1 register
            pub const UART1 = @intToPtr(*volatile types.peripherals.UART1, 0x40003400);
            ///  UART2 register
            pub const UART2 = @intToPtr(*volatile types.peripherals.UART2, 0x40003800);
            ///  UART3 register
            pub const UART3 = @intToPtr(*volatile types.peripherals.UART3, 0x40003c00);
            ///  SPI0 register
            pub const SPI0 = @intToPtr(*volatile types.peripherals.SPI0, 0x40004000);
            ///  SPI1 register
            pub const SPI1 = @intToPtr(*volatile types.peripherals.SPI1, 0x40004400);
            ///  I2C register
            pub const I2C = @intToPtr(*volatile types.peripherals.I2C, 0x40004800);
            ///  PWMx register
            pub const PWMx = @intToPtr(*volatile types.peripherals.PWMx, 0x40005000);
            ///  USB register
            pub const USB = @intToPtr(*volatile types.peripherals.USB, 0x40008000);
            ///  USB2 register
            pub const USB2 = @intToPtr(*volatile types.peripherals.USB2, 0x40008400);
            ///  BLE register
            pub const BLE = @intToPtr(*volatile types.peripherals.BLE, 0x4000c000);
            ///  Program Fast Interrupt Controller
            pub const PFIC = @intToPtr(*volatile types.peripherals.PFIC, 0xe000e000);
            ///  Systick register
            pub const Systick = @intToPtr(*volatile types.peripherals.Systick, 0xe000f000);
        };
    };
};

pub const types = struct {
    pub const peripherals = struct {
        ///  System Control Register
        pub const SYS = extern struct {
            reserved8: [8]u8,
            ///  RWA, system clock configuration, SAM
            R16_CLK_SYS_CFG: mmio.Mmio(packed struct(u16) {
                ///  RWA, output clock divider from PLL or CK32M
                RB_CLK_PLL_DIV: u5,
                reserved6: u1,
                ///  RWA, system clock source mode: 00=divided from 32MHz, 01=divided from PLL-480MHz, 10=directly from 32MHz, 11=directly from 32KHz
                RB_CLK_SYS_MOD: u2,
                padding: u8,
            }),
            ///  RWA, high frequency clock module power control, SAM
            R8_HFCK_PWR_CTRL: mmio.Mmio(packed struct(u8) {
                reserved2: u2,
                ///  RWA, external 32MHz oscillator power control: 0=power down, 1-power on
                RB_CLK_XT32M_PON: u1,
                ///  RWA, external 32MHz oscillator power keep under halt mode: 0=auto stop, 1=keep running
                RB_CLK_XT32M_KEEP: u1,
                ///  RWA, PLL power control: 0=power down, 1-power on
                RB_CLK_PLL_PON: u1,
                padding: u3,
            }),
            reserved12: [1]u8,
            ///  RWA, sleep clock off control byte 0, SAM
            R8_SLP_CLK_OFF0: mmio.Mmio(packed struct(u8) {
                ///  RWA, close TMR0 clock
                RB_SLP_CLK_TMR0: u1,
                ///  RWA, close TMR1 clock
                RB_SLP_CLK_TMR1: u1,
                ///  RWA, close TMR2 clock
                RB_SLP_CLK_TMR2: u1,
                ///  RWA, close TMR3 clock
                RB_SLP_CLK_TMR3: u1,
                ///  RWA, close UART0 clock
                RB_SLP_CLK_UART0: u1,
                ///  RWA, close UART1 clock
                RB_SLP_CLK_UART1: u1,
                ///  RWA, close UART2 clock
                RB_SLP_CLK_UART2: u1,
                ///  RWA, close UART3 clock
                RB_SLP_CLK_UART3: u1,
            }),
            ///  RWA, sleep clock off control byte 1, SAM
            R8_SLP_CLK_OFF1: mmio.Mmio(packed struct(u8) {
                ///  RWA, close SPI0 clock
                RB_SLP_CLK_SPI0: u1,
                ///  RWA, close SPI1 clock
                RB_SLP_CLK_SPI1: u1,
                ///  RWA, close PWMx clock
                RB_SLP_CLK_PWMX: u1,
                ///  RWA, close I2C clock
                RB_SLP_CLK_I2C: u1,
                ///  RWA, close USB clock
                RB_SLP_CLK_USB: u1,
                reserved7: u2,
                ///  RWA, close BLE clock
                RB_SLP_CLK_BLE: u1,
            }),
            ///  RWA, wake control, SAM
            R8_SLP_WAKE_CTRL: mmio.Mmio(packed struct(u8) {
                ///  RWA, enable USB waking
                RB_SLP_USB_WAKE: u1,
                ///  RWA, enable USB2 waking
                RB_SLP_USB2_WAKE: u1,
                reserved3: u1,
                ///  RWA, enable RTC waking
                RB_SLP_RTC_WAKE: u1,
                ///  RWA, enable GPIO waking
                RB_SLP_GPIO_WAKE: u1,
                ///  RWA, enable BAT waking
                RB_SLP_BAT_WAKE: u1,
                ///  RWA, event wakeup mode
                RB_WAKE_EV_MODE: u1,
                padding: u1,
            }),
            ///  RWA, peripherals power down control, SAM
            R8_SLP_POWER_CTRL: mmio.Mmio(packed struct(u8) {
                ///  RWA, wakeup delay time selection
                RB_WAKE_DLY_MOD: u2,
                reserved4: u2,
                ///  RWA, close main SRAM clock
                RB_SLP_CLK_RAMX: u1,
                ///  RWA, close retention 2KB SRAM clock
                RB_SLP_CLK_RAM2K: u1,
                ///  RWA, SRAM retention voltage selection
                RB_RAM_RET_LV: u1,
                padding: u1,
            }),
            reserved24: [8]u8,
            ///  RW, function pin alternate configuration
            R16_PIN_ALTERNATE: mmio.Mmio(packed struct(u16) {
                ///  RW, TMR0 alternate pin enable
                RB_PIN_TMR0: u1,
                ///  RW, TMR1 alternate pin enable
                RB_PIN_TMR1: u1,
                ///  RW, TMR2 alternate pin enable
                RB_PIN_TMR2: u1,
                ///  RW, TMR3 alternate pin enable
                RB_PIN_TMR3: u1,
                ///  RW, RXD0/TXD0 alternate pin enable
                RB_PIN_UART0: u1,
                ///  RW, RXD1/TXD1 alternate pin enable
                RB_PIN_UART1: u1,
                ///  RW, RXD2/TXD2 alternate pin enable
                RB_PIN_UART2: u1,
                ///  RW, RXD3/TXD3 alternate pin enable
                RB_PIN_UART3: u1,
                ///  RW, SCS/SCK0/MOSI/MISO alternate pin enable
                RB_PIN_SPI0: u1,
                reserved10: u1,
                ///  RW, PWM4/PWM5/PWM7/PWM8/PWM9 alternate pin enable
                RB_PIN_PWMX: u1,
                ///  RW, SCL/SDA alternate pin enable
                RB_PIN_I2C: u1,
                ///  RW, DSR/DTR alternate pin enable
                RB_PIN_MODEM: u1,
                ///  RW, interrupt INT24/INT25 alternate pin enable
                RB_PIN_INTX: u1,
                ///  RW, SCL/SDA alternRW, RXD0/RXD0_/TXD0/TXD0_ invert input/output enableate pin enable
                RB_PIN_U0_INV: u1,
                ///  RW, RF antenna switch control output enable
                RB_RF_ANT_SW_EN: u1,
            }),
            ///  RW, analog pin enable and digital input disable
            R16_PIN_ANALOG_IE: mmio.Mmio(packed struct(u16) {
                ///  RW, ADC/TouchKey channel 9/8 digital input disable
                RB_PIN_ADC8_9_IE: u1,
                ///  RW, ADC/TouchKey channel 7/6 digital input disable
                RB_PIN_ADC6_7_IE: u1,
                ///  RW, ADC/TouchKey channel 10 digital input disable
                RB_PIN_ADC10_IE: u1,
                ///  RW, ADC/TouchKey channel 11 digital input disable
                RB_PIN_ADC11_IE: u1,
                ///  RW,USB2 UDP internal pullup resistance enable
                RB_PIN_USB2_DP_PU: u1,
                ///  RW, USB analog I/O enable: 0=analog I/O disable, 1=analog I/O enable
                RB_PIN_USB2_IE: u1,
                ///  RW,USB UDP internal pullup resistance enable
                RB_PIN_USB_DP_PU: u1,
                ///  RW, USB analog I/O enable: 0=analog I/O disable, 1=analog I/O enable
                RB_PIN_USB_IE: u1,
                reserved9: u1,
                ///  RW, ADC/TouchKey channel0 digital input disable: 0=digital input enable, 1=digital input disable
                RB_PIN_ADC0_IE: u1,
                ///  RW, ADC/TouchKey channel1 digital input disable: 0=digital input enable, 1=digital input disable
                RB_PIN_ADC1_IE: u1,
                ///  RW, ADC/TouchKey channel12 digital input disable: 0=digital input enable, 1=digital input disable
                RB_PIN_ADC12_IE: u1,
                ///  RW, ADC/TouchKey channel13 digital input disable: 0=digital input enable, 1=digital input disable
                RB_PIN_ADC13_IE: u1,
                ///  RW, external 32KHz oscillator digital input disable: 0=digital input enable, 1=digital input disable
                RB_PIN_XT32K_IE: u1,
                ///  RW, ADC/TouchKey channel 2/3 digital input disable: 0=digital input enable, 1=digital input disable
                RB_PIN_ADC2_3_IE: u1,
                ///  RW, ADC/TouchKey channel 4/5 digital input disable: 0=digital input enable, 1=digital input disable
                RB_PIN_ADC4_5_IE: u1,
            }),
            reserved32: [4]u8,
            ///  RWA, power plan before sleep instruction, SAM
            R16_POWER_PLAN: mmio.Mmio(packed struct(u16) {
                ///  RWA, power for retention 2KB SRAM
                RB_PWR_XROM: u1,
                ///  RWA, power for retention 2KB SRAM
                RB_PWR_RAM2K: u1,
                ///  RWA, power retention for core and base peripherals
                RB_PWR_CORE: u1,
                ///  RWA, power retention for USB and BLE
                RB_PWR_EXTEND: u1,
                ///  RWA, power for main SRAM
                RB_PWR_RAM30K: u1,
                reserved7: u2,
                ///  RWA, power for system
                RB_PWR_SYS_EN: u1,
                reserved9: u1,
                ///  RWA, DC/DC converter enable: 0=DC/DC disable and bypass, 1=DC/DC enable
                RB_PWR_DCDC_EN: u1,
                ///  RWA, DC/DC converter pre-enable
                RB_PWR_DCDC_PRE: u1,
                ///  RWA, power plan enable, auto clear after sleep executed
                RB_PWR_MUST_0010: u4,
                ///  RWA, must write 0010
                RB_PWR_PLAN_EN: u1,
            }),
            ///  RWA, aux power adjust control, SAM
            R8_AUX_POWER_ADJ: mmio.Mmio(packed struct(u8) {
                ///  RWA, Ultra-Low-Power LDO voltage adjust
                RB_ULPLDO_ADJ: u3,
                reserved7: u4,
                ///  RWA, Ultra-Low-Power LDO voltage adjust
                RB_DCDC_CHARGE: u1,
            }),
            reserved36: [1]u8,
            ///  RWA, battery voltage detector control, SAM
            R8_BAT_DET_CTRL: mmio.Mmio(packed struct(u8) {
                ///  RWA, battery voltage detector enable/select monitor threshold voltage
                RB_BAT_DET_EN__RB_BAT_LOW_VTHX: u1,
                ///  RWA, battery voltage monitor enable under sleep mode
                RB_BAT_MON_EN: u1,
                ///  RWA, interrupt enable for battery lower voltage
                RB_BAT_LOWER_IE: u1,
                ///  RWA, interrupt enable for battery low voltage
                RB_BAT_LOW_IE: u1,
                padding: u4,
            }),
            ///  RWA, battery voltage detector configuration, SAM
            R8_BAT_DET_CFG: mmio.Mmio(packed struct(u8) {
                ///  RWA, select threshold voltage of battery voltage low
                RB_BAT_LOW_VTH: u2,
                padding: u6,
            }),
            ///  RO, battery status
            R8_BAT_STATUS: mmio.Mmio(packed struct(u8) {
                ///  RO, battery lower voltage status, high action
                RB_BAT_STAT_LOWER: u1,
                ///  RO, battery low voltage status, high action
                RB_BAT_STAT_LOW: u1,
                padding: u6,
            }),
            reserved44: [5]u8,
            ///  RWA, internal 32KHz oscillator tune control, SAM
            R16_INT32K_TUNE: mmio.Mmio(packed struct(u16) {
                ///  RWA, internal 32KHz oscillator frequency tune
                RB_INT32K_TUNE: u13,
                padding: u3,
            }),
            ///  RWA, external 32KHz oscillator tune control, SAM
            R8_XT32K_TUNE: mmio.Mmio(packed struct(u8) {
                ///  RWA, external 32KHz oscillator current tune: 00=75% current, 01=standard current, 10=150% current, 11=200% current
                RB_XT32K_I_TUNE: u2,
                reserved4: u2,
                ///  RWA, external 32KHz oscillator load capacitor tune: Cap = RB_XT32K_C_LOAD + 12pF
                RB_XT32K_C_LOAD: u4,
            }),
            ///  RWA, 32KHz oscillator configure
            R8_CK32K_CONFIG: mmio.Mmio(packed struct(u8) {
                ///  RWA, external 32KHz oscillator power on
                RB_CLK_XT32K_PON: u1,
                ///  RWA, internal 32KHz oscillator power on
                RB_CLK_INT32K_PON: u1,
                ///  RWA, 32KHz oscillator source selection: 0=RC, 1=XT
                RB_CLK_OSC32K_XT: u1,
                ///  RWA, internal 32KHz oscillator low noise mode enable
                RB_CLK_OSC32K_FILT: u1,
                reserved7: u3,
                ///  RO, 32KHz oscillator clock pin status
                RB_32K_CLK_PIN: u1,
            }),
            ///  RW, RTC flag and clear control
            R8_RTC_FLAG_CTRL: mmio.Mmio(packed struct(u8) {
                reserved4: u4,
                ///  RW, set 1 to clear RTC timer action flag, auto clear
                RB_RTC_TMR_CLR: u1,
                ///  RW, set 1 to clear RTC trigger action flag, auto clear
                RB_RTC_TRIG_CLR: u1,
                ///  RO, RTC timer action flag
                RB_RTC_TMR_FLAG: u1,
                ///  RO, RTC trigger action flag
                RB_RTC_TRIG_FLAG: u1,
            }),
            ///  RWA, RTC mode control, SAM
            R8_RTC_MODE_CTRL: mmio.Mmio(packed struct(u8) {
                ///  RWA, RTC timer mode: 000=0.125S, 001=0.25S, 010=0.5S, 011=1S, 100=2S, 101=4S, 110=8S, 111=16S
                RB_RTC_TMR_MODE: u3,
                ///  RWA, force ignore bit0 for trigger mode: 0=compare bit0, 1=ignore bit0
                RB_RTC_IGNORE_B0: u1,
                ///  RWA, RTC timer mode enable
                RB_RTC_TMR_EN: u1,
                ///  RWA, RTC trigger mode enable
                RB_RTC_TRIG_EN: u1,
                ///  RWA, set 1 to load RTC count low word R32_RTC_CNT_32K, auto clear after loaded
                RB_RTC_LOAD_LO: u1,
                ///  RWA, set 1 to load RTC count high word R32_RTC_CNT_DAY, auto clear after loaded
                RB_RTC_LOAD_HI: u1,
            }),
            reserved52: [2]u8,
            ///  RWA, RTC trigger value, SAM
            R32_RTC_TRIG: mmio.Mmio(packed struct(u32) {
                ///  RWA, RTC trigger value
                R32_RTC_TRIG: u32,
            }),
            ///  RO, RTC count based 32KHz
            R16_RTC_CNT_32K: mmio.Mmio(packed struct(u16) {
                ///  RWA,RTC count based 32KHz
                R16_RTC_CNT_32K: u16,
            }),
            ///  RO, RTC count based 2 second
            R16_RTC_CNT_2S: mmio.Mmio(packed struct(u16) {
                ///  RO, RTC count based 2 second
                R16_RTC_CNT_2S: u16,
            }),
            ///  RO, RTC count based one day, only low 14 bit
            R32_RTC_CNT_DAY: mmio.Mmio(packed struct(u32) {
                ///  RWA,RTC count based one day
                R32_RTC_CNT_DAY: u14,
                padding: u18,
            }),
            ///  WO, safe accessing sign register, must write SAFE_ACCESS_SIG1 then SAFE_ACCESS_SIG2 to enter safe accessing mode
            R8_SAFE_ACCESS_SIG: mmio.Mmio(packed struct(u8) {
                ///  RO, current safe accessing mode: 11=safe unlocked (SAM), other=locked (00..01..10..11)
                RB_SAFE_ACC_MODE: u2,
                reserved3: u1,
                ///  RO, indicate safe accessing status now: 0=locked, read-only, 1=safe/unlocked (SAM), write enabled
                RB_SAFE_ACC_ACT: u1,
                ///  RO, safe accessing timer bit mask (16*clock number)
                RB_SAFE_ACC_TIMER: u3,
                padding: u1,
            }),
            ///  RF, chip ID register, always is ID_CH58*
            R8_CHIP_ID: mmio.Mmio(packed struct(u8) {
                ///  RF,chip ID register
                R8_CHIP_ID: u8,
            }),
            ///  RF, safe accessing ID register, always 0x0C
            R8_SAFE_ACCESS_ID: mmio.Mmio(packed struct(u8) {
                ///  RF,safe accessing ID register
                R8_SAFE_ACCESS_ID: u8,
            }),
            ///  RW, watch-dog count, count by clock frequency Fsys/131072
            R8_WDOG_COUNT: mmio.Mmio(packed struct(u8) {
                ///  RF,watch-dog count, count by clock frequency Fsys/131072
                R8_WDOG_COUNT: u8,
            }),
            ///  RWA, reset status, SAM or flash ROM configuration
            R8_RESET_STATUS__R8_GLOB_ROM_CFG: mmio.Mmio(packed struct(u8) {
                ///  RO, recent reset flag
                RB_RESET_FLAG: u3,
                reserved4: u1,
                ///  RWA, code offset address selection in Flash ROM: 0=start address 0x000000, 1=start address 0x008000
                RB_ROM_CODE_OFS: u1,
                ///  RWA, enable flash ROM control interface enable
                RB_ROM_CTRL_EN: u1,
                ///  RWA,enable flash ROM data and code area being erase/write
                RB_ROM_DATA_WE: u1,
                ///  RWA, enable flash ROM code area being erase or write
                RB_ROM_CODE_WE: u1,
            }),
            ///  RO, global configuration information and status
            R8_GLOB_CFG_INFO: mmio.Mmio(packed struct(u8) {
                ///  RO, indicate protected status of Flash ROM code and data: 0=reading protect, 1=enable read by external programmer
                RB_CFG_ROM_READ: u1,
                reserved2: u1,
                ///  RO, manual reset input enable status
                RB_CFG_RESET_EN: u1,
                ///  RO, boot-loader enable status
                RB_CFG_BOOT_EN: u1,
                ///  RO, debug enable status
                RB_CFG_DEBUG_EN: u1,
                ///  RO, indicate boot loader status: 0=application status (by software reset), 1=boot loader status
                RB_BOOT_LOADER: u1,
                padding: u2,
            }),
            ///  RWA, reset and watch-dog control, SAM
            R8_RST_WDOG_CTRL: mmio.Mmio(packed struct(u8) {
                ///  WA or WZ, global software reset, high action, auto clear
                RB_SOFTWARE_RESET: u1,
                ///  RWA, enable watch-dog reset if watch-dog timer overflow: 0=as timer only, 1=enable reset if timer overflow
                RB_WDOG_RST_EN: u1,
                ///  RWA, watch-dog timer overflow interrupt enable: 0=disable, 1=enable
                RB_WDOG_INT_EN: u1,
                reserved4: u1,
                ///  RW1, watch-dog timer overflow interrupt flag, cleared by RW1 or reload watch-dog count or __SEV(Send-Event)
                RB_WDOG_INT_FLAG: u1,
                padding: u3,
            }),
            ///  RW, value keeper during global reset
            R8_GLOB_RESET_KEEP: mmio.Mmio(packed struct(u8) {
                ///  RW, value keeper during global reset
                R8_GLOB_RESET_KEEP: u8,
            }),
            reserved75: [3]u8,
            ///  RWA, PLL configuration control, SAM
            R8_PLL_CONFIG: mmio.Mmio(packed struct(u8) {
                ///  RWA, PLL configure data
                RB_PLL_CFG_DAT: u7,
                ///  RWA, flash ROM interface mode
                RB_FLASH_IO_MOD: u1,
            }),
            reserved78: [2]u8,
            ///  RWA, external 32MHz oscillator tune control, SAM
            R8_XT32M_TUNE: mmio.Mmio(packed struct(u8) {
                ///  RWA, external 32MHz oscillator bias current tune: 00=75% current, 01=standard current, 10=125% current, 11=150% current
                RB_XT32M_I_BIAS: u2,
                reserved4: u2,
                ///  RWA, external 32MHz oscillator load capacitor tune: Cap = RB_XT32M_C_LOAD * 2 + 10pF
                RB_XT32M_C_LOAD: u3,
                padding: u1,
            }),
            reserved80: [1]u8,
            ///  RO, system clock count value for 32KHz multi-cycles
            R16_OSC_CAL_CNT: mmio.Mmio(packed struct(u16) {
                ///  RO, system clock count value for 32KHz multi-cycles
                RB_OSC_CAL_CNT: u14,
                ///  RW1, indicate R8_OSC_CAL_OV_CNT not zero, set 1 to clear R8_OSC_CAL_OV_CNT
                RB_OSC_CAL_OV_CLR: u1,
                ///  RW1, interrupt flag for oscillator capture end, set 1 to clear
                RB_OSC_CAL_IF: u1,
            }),
            ///  RO, oscillator frequency calibration overflow times
            R8_OSC_CAL_OV_CNT: mmio.Mmio(packed struct(u8) {
                ///  RO, oscillator frequency calibration overflow times
                R8_OSC_CAL_OV_CNT: u8,
            }),
            ///  RWA, oscillator frequency calibration control, SAM
            R8_OSC_CAL_CTRL: mmio.Mmio(packed struct(u8) {
                ///  RWA, total cycles mode for oscillator capture
                RB_OSC_CNT_TOTAL: u3,
                ///  RO, calibration counter halt status: 0=counting, 1=halt for reading count value
                RB_OSC_CNT_HALT: u1,
                ///  RWA, interrupt enable for oscillator capture end
                RB_OSC_CAL_IE: u1,
                ///  RWA, calibration counter enable
                RB_OSC_CNT_EN: u1,
                ///  RWA, select oscillator capture end mode: 0=normal, 1=append 2 cycles
                RB_OSC_CNT_END: u1,
                padding: u1,
            }),
            ///  RW, Touchkey charge and discharge count
            R8_TKEY_COUNT: mmio.Mmio(packed struct(u8) {
                ///  RW, Touchkey charge count
                RB_TKEY_CHARG_CNT: u5,
                ///  RW, Touchkey discharge count
                RB_TKEY_DISCH_CNT: u3,
            }),
            reserved86: [1]u8,
            ///  RW, Touchkey convert start control
            R8_TKEY_CONVERT: mmio.Mmio(packed struct(u8) {
                ///  RW, Touchkey convert start control
                RB_TKEY_START: u1,
                padding: u7,
            }),
            ///  RW, Touchkey configure
            R8_TKEY_CFG: mmio.Mmio(packed struct(u8) {
                ///  RW, Touchkey power on
                RB_TKEY_PWR_ON: u1,
                ///  RW, Touchkey charge current selection
                RB_TKEY_CURRENT: u1,
                ///  RW, Touchkey drive shield enable
                RB_TKEY_DRV_EN: u1,
                ///  RW, ADC input PGA speed selection
                RB_TKEY_PGA_ADJ: u1,
                padding: u4,
            }),
            ///  RW, ADC input channel selection
            R8_ADC_CHANNEL: mmio.Mmio(packed struct(u8) {
                ///  RW, ADC input channel index
                RB_ADC_CH_INX: u4,
                padding: u4,
            }),
            ///  RW, ADC configure
            R8_ADC_CFG: mmio.Mmio(packed struct(u8) {
                ///  RW, ADC power control: 0=power down, 1=power on
                RB_ADC_POWER_ON: u1,
                ///  RW, ADC input buffer enable
                RB_ADC_BUF_EN: u1,
                ///  RW, ADC input channel mode: 0=single-end, 1=differnetial
                RB_ADC_DIFF_EN: u1,
                ///  RW, enable ADC offset test mode: 0=normal mode, 1=short to test offset
                RB_ADC_OFS_TEST: u1,
                ///  RW, set ADC input PGA gain: 00=-12dB, 01=-6dB, 10=0dB, 11=6dB
                RB_ADC_PGA_GAIN: u2,
                ///  RW, select ADC clock frequency: 00=3.2MHz, 01=2.67MHz, 10=5.33MHz, 11=4MHz
                RB_ADC_CLK_DIV: u2,
            }),
            ///  RW, ADC convert control
            R8_ADC_CONVERT: mmio.Mmio(packed struct(u8) {
                ///  RW, ADC convert start control: 0=stop ADC convert, 1=start an ADC convert, auto clear
                RB_ADC_START: u1,
                reserved7: u6,
                ///  RO, end of ADC conversion flag
                RB_ADC_EOC_X: u1,
            }),
            ///  RW, temperature sensor control
            R8_TEM_SENSOR: mmio.Mmio(packed struct(u8) {
                reserved7: u7,
                ///  RW, temperature sensor power control: 0=power down, 1=power on
                RB_TEM_SEN_PWR_ON: u1,
            }),
            ///  RO, ADC data
            R16_ADC_DATA: mmio.Mmio(packed struct(u16) {
                ///  RO, ADC conversion data
                RB_ADC_DATA: u12,
                padding: u4,
            }),
            ///  RO, ADC interrupt flag register
            R8_ADC_INT_FLAG: mmio.Mmio(packed struct(u8) {
                reserved7: u7,
                ///  RO, ADC conversion interrupt flag: 0=free or converting, 1=end of conversion, interrupt action, write R8_ADC_CONVERT to clear flag
                RB_ADC_IF_EOC: u1,
            }),
            reserved96: [1]u8,
            ///  RO, ADC DMA control and status register
            R32_ADC_DMA_CTRL: mmio.Mmio(packed struct(u8) {
                ///  RW, ADC DMA enable
                R32_ADC_DMA_CTRL: u8,
            }),
            ///  RW, ADC DMA control
            R8_ADC_CTRL_DMA: mmio.Mmio(packed struct(u8) {
                ///  RW, ADC DMA enable
                RB_ADC_DMA_ENABLE: u1,
                reserved2: u1,
                ///  RW, ADC DMA address loop enable
                RB_ADC_DMA_LOOP: u1,
                ///  RW, enable interrupt for ADC DMA completion
                RB_ADC_IE_DMA_END: u1,
                ///  RW, enable interrupt for end of ADC conversion
                RB_ADC_IE_EOC: u1,
                reserved6: u1,
                ///  RW, enable contineous conversion ADC
                RB_ADC_CONT_EN: u1,
                ///  RW, enable auto continuing ADC for DMA
                RB_ADC_AUTO_EN: u1,
            }),
            ///  RO, ADC interrupt flag
            R8_ADC_DMA_IF: mmio.Mmio(packed struct(u8) {
                reserved3: u3,
                ///  interrupt flag for ADC DMA completion
                RB_ADC_IF_DMA_END: u1,
                ///  interrupt flag for end of ADC conversion
                RB_ADC_IF_END_ADC: u1,
                padding: u3,
            }),
            ///  RO, ADC interrupt flag
            R8_ADC_AUTO_CYCLE: mmio.Mmio(packed struct(u8) {
                ///  auto ADC cycle value, unit is 16 Fsys
                R8_ADC_AUTO_CYCLE: u8,
            }),
            ///  RO, ADC DMA current address
            R16_ADC_DMA_NOW: mmio.Mmio(packed struct(u16) {
                ///  ADC DMA current address
                R16_ADC_DMA_NOW: u16,
            }),
            reserved104: [2]u8,
            ///  RW, ADC DMA begin address
            R16_ADC_DMA_BEG: mmio.Mmio(packed struct(u16) {
                ///  ADC DMA begin address
                R16_ADC_DMA_BEG: u16,
            }),
            reserved108: [2]u8,
            ///  RW, ADC DMA end address
            R16_ADC_DMA_END: mmio.Mmio(packed struct(u16) {
                ///  ADC DMA end address
                R16_ADC_DMA_END: u16,
            }),
            reserved144: [34]u8,
            ///  RW, GPIO PA interrupt enable
            R16_PA_INT_EN: mmio.Mmio(packed struct(u16) {
                ///  GPIO PA interrupt enable
                R16_PA_INT_EN: u16,
            }),
            ///  RW, GPIO PB interrupt enable
            R16_PB_INT_EN: mmio.Mmio(packed struct(u16) {
                ///  GPIO PB interrupt enable
                R16_PB_INT_EN: u16,
            }),
            ///  RW, GPIO PA interrupt mode: 0=level action, 1=edge action
            R16_PA_INT_MODE: mmio.Mmio(packed struct(u16) {
                ///  GPIO PA interrupt mode
                R16_PA_INT_MODE: u16,
            }),
            ///  RW, GPIO PB interrupt mode: 0=level action, 1=edge action;RW, status for parallel slave read
            R16_PB_INT_MODE: mmio.Mmio(packed struct(u16) {
                ///  GPIO PB interrupt mode
                R16_PB_INT_MODE: u16,
            }),
            reserved156: [4]u8,
            ///  RW1, GPIO PA interrupt flag
            R16_PA_INT_IF: mmio.Mmio(packed struct(u16) {
                ///  GPIO PA interrupt flag
                R16_PA_INT_IF: u16,
            }),
            ///  RW1, GPIO PB interrupt flag
            R16_PB_INT_IF: mmio.Mmio(packed struct(u16) {
                ///  GPIO PB interrupt flag
                R16_PB_INT_IF: u16,
            }),
            ///  RW, GPIO PA I/O direction: 0=in, 1=out
            R32_PA_DIR: mmio.Mmio(packed struct(u32) {
                ///  GPIO PA I/O direction byte 0
                R8_PA_DIR_0: u8,
                ///  GPIO PA I/O direction byte 1
                R8_PA_DIR_1: u8,
                padding: u16,
            }),
            ///  RO, GPIO PA input
            R32_PA_PIN: mmio.Mmio(packed struct(u32) {
                ///  GPIO PA input byte 0
                R8_PA_PIN_0: u8,
                ///  GPIO PA input byte 1
                R8_PA_PIN_1: u8,
                padding: u16,
            }),
            ///  RW, GPIO PA output
            R32_PA_OUT: mmio.Mmio(packed struct(u32) {
                ///  GPIO PA output byte 0
                R8_PA_OUT_0: u8,
                ///  GPIO PA output byte 1
                R8_PA_OUT_1: u8,
                padding: u16,
            }),
            ///  WZ, GPIO PA clear output: 0=keep, 1=clear
            R32_PA_CLR: mmio.Mmio(packed struct(u32) {
                ///  GPIO PA clear output byte 0
                R8_PA_CLR_0: u8,
                ///  GPIO PA clear output byte 1
                R8_PA_CLR_1: u8,
                padding: u16,
            }),
            ///  RW, GPIO PA pullup resistance enable
            R32_PA_PU: mmio.Mmio(packed struct(u32) {
                ///  GPIO PA pullup resistance enable byte 0
                R8_PA_PU_0: u8,
                ///  GPIO PA pullup resistance enable byte 0
                R8_PA_PU_1: u8,
                padding: u16,
            }),
            ///  RW, PA pulldown for input or PA driving capability for output
            R32_PA_PD_DRV: mmio.Mmio(packed struct(u32) {
                ///  PA pulldown for input or PA driving capability for output byte 0
                R8_PA_PD_DRV_0: u8,
                ///  PA pulldown for input or PA driving capability for output byte 1
                R8_PA_PD_DRV_1: u8,
                padding: u16,
            }),
            reserved192: [8]u8,
            ///  RW, GPIO PB I/O direction: 0=in, 1=out
            R32_PB_DIR: mmio.Mmio(packed struct(u32) {
                ///  GPIO PB I/O direction byte 0
                R8_PB_DIR_0: u8,
                ///  GPIO PB I/O direction byte 1
                R8_PB_DIR_1: u8,
                ///  GPIO PB I/O direction byte 2
                R8_PB_DIR_2: u8,
                padding: u8,
            }),
            ///  RO, GPIO PB input
            R32_PB_PIN: mmio.Mmio(packed struct(u32) {
                ///  GPIO PB input byte 0
                R8_PB_PIN_0: u8,
                ///  GPIO PB input byte 1
                R8_PB_PIN_1: u8,
                ///  GPIO PB input byte 2
                R8_PB_PIN_2: u8,
                padding: u8,
            }),
            ///  RW, GPIO PB output;RW, data for parallel slave read
            R32_PB_OUT__R8_SLV_RD_DATA: mmio.Mmio(packed struct(u32) {
                ///  GPIO PB output byte 0
                R8_PB_OUT_0: u8,
                ///  GPIO PB output byte 1
                R8_PB_OUT_1: u8,
                ///  GPIO PB output byte 2
                R8_PB_OUT_2: u8,
                padding: u8,
            }),
            ///  WZ, GPIO PB clear output: 0=keep, 1=clear
            R32_PB_CLR: mmio.Mmio(packed struct(u32) {
                ///  GPIO PB clear output byte 0
                R8_PB_CLR_0: u8,
                ///  GPIO PB clear output byte 1
                R8_PB_CLR_1: u8,
                ///  GPIO PB clear output byte 2
                R8_PB_CLR_2: u8,
                padding: u8,
            }),
            ///  RW, GPIO PB pullup resistance enable
            R32_PB_PU: mmio.Mmio(packed struct(u32) {
                ///  GPIO PB pullup resistance enable byte 0
                R8_PB_PU_0: u8,
                ///  GPIO PB pullup resistance enable byte 1
                R8_PB_PU_1: u8,
                ///  GPIO PB pullup resistance enable byte 2
                R8_PB_PU_2: u8,
                padding: u8,
            }),
            ///  RW, PB pulldown for input or PB driving capability for output
            R32_PB_PD_DRV: mmio.Mmio(packed struct(u32) {
                ///  PB pulldown for input or PB driving capability for output byte 0
                R8_PB_PD_DRV_0: u8,
                ///  PB pulldown for input or PB driving capability for output byte 0
                R8_PB_PD_DRV_1: u8,
                ///  PB pulldown for input or PB driving capability for output byte 0
                R8_PB_PD_DRV_2: u8,
                padding: u8,
            }),
        };

        ///  Timer0 register
        pub const TMR0 = extern struct {
            ///  RW, TMR0 mode control
            R8_TMR0_CTRL_MOD: mmio.Mmio(packed struct(u8) {
                ///  RW, timer in mode: 0=timer/PWM, 1=capture/count
                RB_TMR_MODE_IN: u1,
                ///  RW, force clear timer FIFO and count
                RB_TMR_ALL_CLEAR: u1,
                ///  RW, timer count enable
                RB_TMR_COUNT_EN: u1,
                ///  RW, timer output enable
                RB_TMR_OUT_EN: u1,
                ///  RW, timer PWM output polarity: 0=default low and high action, 1=default high and low action;RW, count sub-mode if RB_TMR_MODE_IN=1: 0=capture, 1=count
                RB_TMR_OUT_POLAR__RB_TMR_CAP_COUNT: u1,
                reserved6: u1,
                ///  RW, timer PWM repeat mode: 00=1, 01=4, 10=8, 11-16;RW, timer capture edge mode: 00=disable, 01=edge change, 10=fall to fall, 11-rise to rise
                RB_TMR_PWM_REPEAT__RB_TMR_CAP_EDGE: u2,
            }),
            reserved2: [1]u8,
            ///  RW, TMR0 interrupt enable
            R8_TMR0_INTER_EN: mmio.Mmio(packed struct(u8) {
                ///  RW, enable interrupt for timer capture count timeout or PWM cycle end
                RB_TMR_IE_CYC_END: u1,
                ///  RW, enable interrupt for timer capture input action or PWM trigger
                RB_TMR_IE_DATA_ACT: u1,
                ///  RW, enable interrupt for timer FIFO half (capture fifo >=4 or PWM fifo less than 3)
                RB_TMR_IE_FIFO_HF: u1,
                ///  RW, enable interrupt for timer1/2 DMA completion
                RB_TMR_IE_DMA_END: u1,
                ///  RW, enable interrupt for timer FIFO overflow
                RB_TMR_IE_FIFO_OV: u1,
                padding: u3,
            }),
            reserved6: [3]u8,
            ///  RW1, TMR0 interrupt flag
            R8_TMR0_INT_FLAG: mmio.Mmio(packed struct(u8) {
                ///  RW1, interrupt flag for timer capture count timeout or PWM cycle end
                RB_TMR_IF_CYC_END: u1,
                ///  RW1, interrupt flag for timer capture input action or PWM trigger
                RB_TMR_IF_DATA_ACT: u1,
                ///  RW1, interrupt flag for timer FIFO half (capture fifo >=4 or PWM fifo less than 3
                RB_TMR_IF_FIFO_HF: u1,
                ///  RW1, interrupt flag for timer1/2 DMA completion
                RB_TMR_IF_DMA_END: u1,
                ///  RW1, interrupt flag for timer FIFO overflow
                RB_TMR_IF_FIFO_OV: u1,
                padding: u3,
            }),
            ///  RO, TMR0 FIFO count status
            R8_TMR0_FIFO_COUNT: mmio.Mmio(packed struct(u8) {
                ///  RW1,TMR0 FIFO count status
                R8_TMR0_FIFO_COUNT: u8,
            }),
            ///  RO, TMR0 current count
            R32_TMR0_COUNT: mmio.Mmio(packed struct(u32) {
                ///  RW1,TMR0 current count
                R32_TMR0_COUNT: u32,
            }),
            ///  RW, TMR0 end count value, only low 26 bit
            R32_TMR0_CNT_END: mmio.Mmio(packed struct(u32) {
                ///  RW1,TMR0 end count value
                R32_TMR0_CNT_END: u32,
            }),
            ///  RO/WO, TMR0 FIFO register, only low 26 bit
            R32_TMR0_FIFO: mmio.Mmio(packed struct(u32) {
                ///  RW1,TMR0 FIFO register
                R32_TMR0_FIFO: u32,
            }),
        };

        ///  Timer1 register
        pub const TMR1 = extern struct {
            ///  RW, TMR1 mode control
            R8_TMR1_CTRL_MOD: mmio.Mmio(packed struct(u8) {
                ///  RW, timer in mode: 0=timer/PWM, 1=capture/count
                RB_TMR_MODE_IN: u1,
                ///  RW, force clear timer FIFO and count
                RB_TMR_ALL_CLEAR: u1,
                ///  RW, timer count enable
                RB_TMR_COUNT_EN: u1,
                ///  RW, timer output enable
                RB_TMR_OUT_EN: u1,
                ///  RW, timer PWM output polarity: 0=default low and high action, 1=default high and low action;RW, count sub-mode if RB_TMR_MODE_IN=1: 0=capture, 1=count
                RB_TMR_OUT_POLAR__RB_TMR_CAP_COUNT: u1,
                reserved6: u1,
                ///  RW, timer PWM repeat mode: 00=1, 01=4, 10=8, 11-16;RW, timer capture edge mode: 00=disable, 01=edge change, 10=fall to fall, 11-rise to rise
                RB_TMR_PWM_REPEAT__RB_TMR_CAP_EDGE: u2,
            }),
            ///  RW, TMR1 DMA control
            R8_TMR1_CTRL_DMA: mmio.Mmio(packed struct(u8) {
                ///  RW, timer1/2 DMA enable
                RB_TMR_DMA_ENABLE: u1,
                reserved2: u1,
                ///  RW, timer1/2 DMA address loop enable
                RB_TMR_DMA_LOOP: u1,
                padding: u5,
            }),
            ///  RW, TMR1 interrupt enable
            R8_TMR1_INTER_EN: mmio.Mmio(packed struct(u8) {
                ///  RW, enable interrupt for timer capture count timeout or PWM cycle end
                RB_TMR_IE_CYC_END: u1,
                ///  RW, enable interrupt for timer capture input action or PWM trigger
                RB_TMR_IE_DATA_ACT: u1,
                ///  RW, enable interrupt for timer FIFO half (capture fifo >=4 or PWM fifo less than 3)
                RB_TMR_IE_FIFO_HF: u1,
                ///  RW, enable interrupt for timer1/2 DMA completion
                RB_TMR_IE_DMA_END: u1,
                ///  RW, enable interrupt for timer FIFO overflow
                RB_TMR_IE_FIFO_OV: u1,
                padding: u3,
            }),
            reserved6: [3]u8,
            ///  RW1, TMR1 interrupt flag
            R8_TMR1_INT_FLAG: mmio.Mmio(packed struct(u8) {
                ///  RW1, interrupt flag for timer capture count timeout or PWM cycle end
                RB_TMR_IF_CYC_END: u1,
                ///  RW1, interrupt flag for timer capture input action or PWM trigger
                RB_TMR_IF_DATA_ACT: u1,
                ///  RW1, interrupt flag for timer FIFO half (capture fifo >=4 or PWM fifo less than 3
                RB_TMR_IF_FIFO_HF: u1,
                ///  RW1, interrupt flag for timer1/2 DMA completion
                RB_TMR_IF_DMA_END: u1,
                ///  RW1, interrupt flag for timer FIFO overflow
                RB_TMR_IF_FIFO_OV: u1,
                padding: u3,
            }),
            ///  RO, TMR1 FIFO count status
            R8_TMR1_FIFO_COUNT: mmio.Mmio(packed struct(u8) {
                ///  RW1, TMR1 FIFO count status
                R8_TMR1_FIFO_COUNT: u8,
            }),
            ///  RO, TMR1 current count
            R32_TMR1_COUNT: mmio.Mmio(packed struct(u32) {
                ///  RW1,TMR1 current count
                R32_TMR1_COUNT: u32,
            }),
            ///  RW, TMR1 end count value, only low 26 bit
            R32_TMR1_CNT_END: mmio.Mmio(packed struct(u32) {
                ///  RW1,TMR1 end count value,
                R32_TMR1_CNT_END: u32,
            }),
            ///  RO, TMR1 FIFO register, only low 26 bit
            R32_TMR1_FIFO: mmio.Mmio(packed struct(u32) {
                ///  RW1,TMR1 FIFO register
                R32_TMR1_FIFO: u32,
            }),
            ///  RO, TMR1 DMA current address
            R16_TMR1_DMA_NOW: mmio.Mmio(packed struct(u16) {
                ///  RW1,TMR1 FIFO register
                R16_TMR1_DMA_NOW: u16,
            }),
            reserved24: [2]u8,
            ///  RW, TMR1 DMA begin address
            R16_TMR1_DMA_BEG: mmio.Mmio(packed struct(u16) {
                ///  RW1,TMR1 FIFO register
                R16_TMR1_DMA_BEG: u16,
            }),
            reserved28: [2]u8,
            ///  RW, TMR1 DMA end address
            R16_TMR1_DMA_END: mmio.Mmio(packed struct(u16) {
                ///  RW1,TMR1 FIFO register
                R16_TMR1_DMA_END: u16,
            }),
        };

        ///  Timer2 register
        pub const TMR2 = extern struct {
            ///  RW, TMR2 mode control
            R8_TMR2_CTRL_MOD: mmio.Mmio(packed struct(u8) {
                ///  RW, timer in mode: 0=timer_PWM, 1=capture_count
                RB_TMR_MODE_IN: u1,
                ///  RW, force clear timer FIFO and count
                RB_TMR_ALL_CLEAR: u1,
                ///  RW, timer count enable
                RB_TMR_COUNT_EN: u1,
                ///  RW, timer output enable
                RB_TMR_OUT_EN: u1,
                ///  RW, timer PWM output polarity: 0=default low and high action, 1=default high and low action;RW, count sub-mode if RB_TMR_MODE_IN=1: 0=capture, 1=count
                RB_TMR_OUT_POLAR__RB_TMR_CAP_COUNT: u1,
                reserved6: u1,
                ///  RW, timer PWM repeat mode: 00=1, 01=4, 10=8, 11-16;RW, timer capture edge mode: 00=disable, 01=edge change, 10=fall to fall, 11-rise to rise
                RB_TMR_PWM_REPEAT__RB_TMR_CAP_EDGE: u2,
            }),
            ///  RW, TMR2 DMA control
            R8_TMR2_CTRL_DMA: mmio.Mmio(packed struct(u8) {
                ///  RW, timer1_2 DMA enable
                RB_TMR_DMA_ENABLE: u1,
                reserved2: u1,
                ///  RW, timer1_2 DMA address loop enable
                RB_TMR_DMA_LOOP: u1,
                padding: u5,
            }),
            ///  RW, TMR2 interrupt enable
            R8_TMR2_INTER_EN: mmio.Mmio(packed struct(u8) {
                ///  RW, enable interrupt for timer capture count timeout or PWM cycle end
                RB_TMR_IE_CYC_END: u1,
                ///  RW, enable interrupt for timer capture input action or PWM trigger
                RB_TMR_IE_DATA_ACT: u1,
                ///  RW, enable interrupt for timer FIFO half (capture fifo >=4 or PWM fifo less than 3)
                RB_TMR_IE_FIFO_HF: u1,
                ///  RW, enable interrupt for timer1/2 DMA completion
                RB_TMR_IE_DMA_END: u1,
                ///  RW, enable interrupt for timer FIFO overflow
                RB_TMR_IE_FIFO_OV: u1,
                padding: u3,
            }),
            reserved6: [3]u8,
            ///  RW1, TMR2 interrupt flag
            R8_TMR2_INT_FLAG: mmio.Mmio(packed struct(u8) {
                ///  RW1, interrupt flag for timer capture count timeout or PWM cycle end
                RB_TMR_IF_CYC_END: u1,
                ///  RW1, interrupt flag for timer capture input action or PWM trigger
                RB_TMR_IF_DATA_ACT: u1,
                ///  RW1, interrupt flag for timer FIFO half (capture fifo >=4 or PWM fifo less than 3
                RB_TMR_IF_FIFO_HF: u1,
                ///  RW1, interrupt flag for timer1/2 DMA completion
                RB_TMR_IF_DMA_END: u1,
                ///  RW1, interrupt flag for timer FIFO overflow
                RB_TMR_IF_FIFO_OV: u1,
                padding: u3,
            }),
            ///  RO, TMR2 FIFO count status
            R8_TMR2_FIFO_COUNT: mmio.Mmio(packed struct(u8) {
                ///  RW, TMR2 FIFO count status
                R8_TMR2_FIFO_COUNT: u8,
            }),
            ///  RO, TMR2 current count
            R32_TMR2_COUNT: mmio.Mmio(packed struct(u32) {
                ///  RW, TMR2 current count
                R32_TMR2_COUNT: u32,
            }),
            ///  RW, TMR2 end count value, only low 26 bit
            R32_TMR2_CNT_END: mmio.Mmio(packed struct(u32) {
                ///  RW, TMR2 current count
                R32_TMR2_CNT_END: u32,
            }),
            ///  RO, TMR2 FIFO register, only low 26 bit
            R32_TMR2_FIFO: mmio.Mmio(packed struct(u32) {
                ///  RW, TMR2 current count
                R32_TMR2_FIFO: u32,
            }),
            ///  RO, TMR2 DMA current address
            R16_TMR2_DMA_NOW: mmio.Mmio(packed struct(u16) {
                ///  RW, TMR2 current count
                R16_TMR2_DMA_NOW: u16,
            }),
            reserved24: [2]u8,
            ///  RW, TMR2 DMA begin address
            R16_TMR2_DMA_BEG: mmio.Mmio(packed struct(u16) {
                ///  RW, TMR2 DMA begin address
                R16_TMR2_DMA_BEG: u16,
            }),
            reserved28: [2]u8,
            ///  RW, TMR2 DMA end address
            R16_TMR2_DMA_END: mmio.Mmio(packed struct(u16) {
                ///  RW, TMR2 DMA end address
                R16_TMR2_DMA_END: u16,
            }),
        };

        ///  Timer3 register
        pub const TMR3 = extern struct {
            ///  RW, TMR3 mode control
            R8_TMR3_CTRL_MOD: mmio.Mmio(packed struct(u8) {
                ///  RW, timer in mode: 0=timer/PWM, 1=capture/count
                RB_TMR_MODE_IN: u1,
                ///  RW, force clear timer FIFO and count
                RB_TMR_ALL_CLEAR: u1,
                ///  RW, timer count enable
                RB_TMR_COUNT_EN: u1,
                ///  RW, timer output enable
                RB_TMR_OUT_EN: u1,
                ///  RW, timer PWM output polarity: 0=default low and high action, 1=default high and low action;RW, count sub-mode if RB_TMR_MODE_IN=1: 0=capture, 1=count
                RB_TMR_OUT_POLAR__RB_TMR_CAP_COUNT: u1,
                reserved6: u1,
                ///  RW, timer PWM repeat mode: 00=1, 01=4, 10=8, 11-16;RW, timer capture edge mode: 00=disable, 01=edge change, 10=fall to fall, 11-rise to rise
                RB_TMR_PWM_REPEAT__RB_TMR_CAP_EDGE: u2,
            }),
            reserved2: [1]u8,
            ///  RW, TMR3 interrupt enable
            R8_TMR3_INTER_EN: mmio.Mmio(packed struct(u8) {
                ///  RW, enable interrupt for timer capture count timeout or PWM cycle end
                RB_TMR_IE_CYC_END: u1,
                ///  RW, enable interrupt for timer capture input action or PWM trigger
                RB_TMR_IE_DATA_ACT: u1,
                ///  RW, enable interrupt for timer FIFO half (capture fifo >=4 or PWM fifo less than 3)
                RB_TMR_IE_FIFO_HF: u1,
                ///  RW, enable interrupt for timer1/2 DMA completion
                RB_TMR_IE_DMA_END: u1,
                ///  RW, enable interrupt for timer FIFO overflow
                RB_TMR_IE_FIFO_OV: u1,
                padding: u3,
            }),
            reserved6: [3]u8,
            ///  RW1, TMR3 interrupt flag
            R8_TMR3_INT_FLAG: mmio.Mmio(packed struct(u8) {
                ///  RW1, interrupt flag for timer capture count timeout or PWM cycle end
                RB_TMR_IF_CYC_END: u1,
                ///  RW1, interrupt flag for timer capture input action or PWM trigger
                RB_TMR_IF_DATA_ACT: u1,
                ///  RW1, interrupt flag for timer FIFO half (capture fifo >=4 or PWM fifo less than 3
                RB_TMR_IF_FIFO_HF: u1,
                ///  RW1, interrupt flag for timer1/2 DMA completion
                RB_TMR_IF_DMA_END: u1,
                ///  RW1, interrupt flag for timer FIFO overflow
                RB_TMR_IF_FIFO_OV: u1,
                padding: u3,
            }),
            ///  RO, TMR3 FIFO count status
            R8_TMR3_FIFO_COUNT: mmio.Mmio(packed struct(u8) {
                ///  R0, TMR3 FIFO count status
                R8_TMR3_FIFO_COUNT: u8,
            }),
            ///  RO, TMR3 current count
            R32_TMR3_COUNT: mmio.Mmio(packed struct(u32) {
                ///  R0, TMR3 current count
                R32_TMR3_COUNT: u32,
            }),
            ///  RW, TMR3 end count value, only low 26 bit
            R32_TMR3_CNT_END: mmio.Mmio(packed struct(u32) {
                ///  RW, TMR3 end count value, only low 26 bit
                R32_TMR3_CNT_END: u32,
            }),
            ///  RO/WO, TMR3 FIFO register, only low 26 bit
            R32_TMR3_FIFO: mmio.Mmio(packed struct(u32) {
                ///  RO/WO, TMR3 FIFO register, only low 26 bit
                R32_TMR3_FIFO: u32,
            }),
        };

        ///  UART0 register
        pub const UART0 = extern struct {
            ///  RW, UART0 modem control
            R8_UART0_MCR: mmio.Mmio(packed struct(u8) {
                ///  RW, UART0 control DTR
                RB_MCR_DTR: u1,
                ///  RW, UART0 control RTS
                RB_MCR_RTS: u1,
                ///  RW, UART0 control OUT1
                RB_MCR_OUT1: u1,
                ///  RW, UART control OUT2/ UART interrupt output enable
                RB_MCR_OUT2__RB_MCR_INT_OE: u1,
                ///  RW, UART0 enable local loop back
                RB_MCR_LOOP: u1,
                ///  RW, UART0 enable autoflow control
                RB_MCR_AU_FLOW_EN: u1,
                ///  RW, UART0 enable TNOW output on DTR pin
                RB_MCR_TNOW: u1,
                ///  RW, UART0 enable half-duplex
                RB_MCR_HALF: u1,
            }),
            ///  RW, UART0 interrupt enable
            R8_UART0_IER: mmio.Mmio(packed struct(u8) {
                ///  RW, UART interrupt enable for receiver data ready
                RB_IER_RECV_RDY: u1,
                ///  RW, UART interrupt enable for THR empty
                RB_IER_THR_EMPTY: u1,
                ///  RW, UART interrupt enable for receiver line status
                RB_IER_LINE_STAT: u1,
                ///  RW, UART0 interrupt enable for modem status change
                RB_IER_MODEM_CHG: u1,
                ///  RW, UART0 DTR/TNOW output pin enable
                RB_IER_DTR_EN: u1,
                ///  RW, UART0 RTS output pin enable
                RB_IER_RTS_EN: u1,
                ///  RW, UART TXD pin enable
                RB_IER_TXD_EN: u1,
                ///  WZ, UART software reset control, high action, auto clear
                RB_IER_RESET: u1,
            }),
            ///  RW, UART0 FIFO control
            R8_UART0_FCR: mmio.Mmio(packed struct(u8) {
                ///  RW, UART FIFO enable
                RB_FCR_FIFO_EN: u1,
                ///  WZ, clear UART receiver FIFO, high action, auto clear
                RB_FCR_RX_FIFO_CLR: u1,
                ///  WZ, clear UART transmitter FIFO, high action, auto clear
                RB_FCR_TX_FIFO_CLR: u1,
                reserved6: u3,
                ///  RW, UART receiver FIFO trigger level: 00-1byte, 01-2bytes, 10-4bytes, 11-7bytes
                RB_FCR_FIFO_TRIG: u2,
            }),
            ///  RW, UART0 line control
            R8_UART0_LCR: mmio.Mmio(packed struct(u8) {
                ///  RW, UART word bit length: 00-5bit, 01-6bit, 10-7bit, 11-8bit
                RB_LCR_WORD_SZ: u2,
                ///  RW, UART stop bit length: 0-1bit, 1-2bit
                RB_LCR_STOP_BIT: u1,
                ///  RW, UART parity enable
                RB_LCR_PAR_EN: u1,
                ///  RW, UART parity mode: 00-odd, 01-even, 10-mark, 11-space
                RB_LCR_PAR_MOD: u2,
                ///  RW, UART break control enable
                RB_LCR_BREAK_EN: u1,
                ///  RW, UART general purpose bit;RW, UART reserved bit
                RB_LCR_GP_BIT__RB_LCR_DLAB: u1,
            }),
            ///  RO, UART0 interrupt identification
            R8_UART0_IIR: mmio.Mmio(packed struct(u8) {
                ///  RO, UART no interrupt flag: 0=interrupt action, 1=no interrupt
                RB_IIR_NO_INT: u1,
                reserved6: u5,
                ///  RO, UART FIFO enabled flag
                RB_IIR_FIFO_ID: u2,
            }),
            ///  RO, UART0 line status
            R8_UART0_LSR: mmio.Mmio(packed struct(u8) {
                ///  RO, UART receiver fifo data ready status
                RB_LSR_DATA_RDY: u1,
                ///  RZ, UART receiver overrun error
                RB_LSR_OVER_ERR: u1,
                ///  RZ, UART receiver parity error
                RB_LSR_PAR_ERR: u1,
                ///  RZ, UART receiver frame error
                RB_LSR_FRAME_ERR: u1,
                ///  RZ, UART receiver break error
                RB_LSR_BREAK_ERR: u1,
                ///  RO, UART transmitter fifo empty status
                RB_LSR_TX_FIFO_EMP: u1,
                ///  RO, UART transmitter all empty status
                RB_LSR_TX_ALL_EMP: u1,
                ///  RO, indicate error in UART receiver fifo
                RB_LSR_ERR_RX_FIFO: u1,
            }),
            ///  RO, UART0 modem status
            R8_UART0_MSR: mmio.Mmio(packed struct(u8) {
                ///  RZ, UART0 CTS changed status, high action
                RB_MSR_CTS_CHG: u1,
                ///  RZ, UART0 DSR changed status, high action
                RB_MSR_DSR_CHG: u1,
                ///  RZ, UART0 RI changed status, high action
                RB_MSR_RI_CHG: u1,
                ///  RZ, UART0 DCD changed status, high action
                RB_MSR_DCD_CHG: u1,
                ///  RO, UART0 CTS action status
                RB_MSR_CTS: u1,
                ///  RO, UART0 DSR action statusv
                RB_MSR_DSR: u1,
                ///  RO, UART0 RI action status
                RB_MSR_RI: u1,
                ///  RO, UART0 DCD action status
                RB_MSR_DCD: u1,
            }),
            reserved8: [1]u8,
            ///  RO, UART0 receiver buffer, receiving byte
            R8_UART0_RBR: mmio.Mmio(packed struct(u8) {
                ///  RO, UART0 receiver buffer, receiving byte
                R8_UART0_RBR: u8,
            }),
            reserved10: [1]u8,
            ///  RO, UART0 receiver FIFO count
            R8_UART0_RFC: mmio.Mmio(packed struct(u8) {
                ///  RO, UART0 receiver FIFO count
                R8_UART0_RFC: u8,
            }),
            ///  RO, UART0 transmitter FIFO count
            R8_UART0_TFC: mmio.Mmio(packed struct(u8) {
                ///  RO, UART0 transmitter FIFO count
                R8_UART0_TFC: u8,
            }),
            ///  RW, UART0 divisor latch
            R16_UART0_DL: mmio.Mmio(packed struct(u16) {
                ///  RW, UART0 divisor latch
                R16_UART0_DL: u16,
            }),
            ///  RW, UART0 pre-divisor latch byte, only low 7 bit, from 1 to 0/128
            R8_UART0_DIV: mmio.Mmio(packed struct(u8) {
                ///  RW,UART0 pre-divisor latch byte, only low 7 bit, from 1 to 0/128
                R8_UART0_DIV: u8,
            }),
            ///  RW, UART0 slave address: 0xFF=disable, other=enable
            R8_UART0_ADR: mmio.Mmio(packed struct(u8) {
                ///  RW,UART0 slave address: 0xFF=disable, other=enable
                R8_UART0_ADR: u8,
            }),
        };

        ///  UART1 register
        pub const UART1 = extern struct {
            ///  RW, UART1 modem control
            R8_UART1_MCR: mmio.Mmio(packed struct(u8) {
                reserved3: u3,
                ///  RW, UART control OUT2/UART interrupt output enable
                RB_MCR_OUT2__RB_MCR_INT_OE: u1,
                padding: u4,
            }),
            ///  RW, UART1 interrupt enable
            R8_UART1_IER: mmio.Mmio(packed struct(u8) {
                ///  RW, UART interrupt enable for receiver data ready
                RB_IER_RECV_RDY: u1,
                ///  RW, UART interrupt enable for THR empty
                RB_IER_THR_EMPTY: u1,
                ///  RW, UART interrupt enable for receiver line status
                RB_IER_LINE_STAT: u1,
                reserved6: u3,
                ///  RW, UART TXD pin enable
                RB_IER_TXD_EN: u1,
                ///  WZ, UART software reset control, high action, auto clear
                RB_IER_RESET: u1,
            }),
            ///  RW, UART1 FIFO control
            R8_UART1_FCR: mmio.Mmio(packed struct(u8) {
                ///  RW, UART FIFO enable
                RB_FCR_FIFO_EN: u1,
                ///  WZ, clear UART receiver FIFO, high action, auto clear
                RB_FCR_RX_FIFO_CLR: u1,
                ///  WZ, clear UART transmitter FIFO, high action, auto clear
                RB_FCR_TX_FIFO_CLR: u1,
                reserved6: u3,
                ///  RW, UART receiver FIFO trigger level: 00-1byte, 01-2bytes, 10-4bytes, 11-7bytes
                RB_FCR_FIFO_TRIG: u2,
            }),
            ///  RW, UART1 line control
            R8_UART1_LCR: mmio.Mmio(packed struct(u8) {
                ///  RW, UART word bit length: 00-5bit, 01-6bit, 10-7bit, 11-8bit
                RB_LCR_WORD_SZ: u2,
                ///  RW, UART stop bit length: 0-1bit, 1-2bit
                RB_LCR_STOP_BIT: u1,
                ///  RW, UART parity enable
                RB_LCR_PAR_EN: u1,
                ///  RW, UART parity mode: 00-odd, 01-even, 10-mark, 11-space
                RB_LCR_PAR_MOD: u2,
                ///  RW, UART break control enable
                RB_LCR_BREAK_EN: u1,
                ///  RW, UART general purpose bit;RW, UART reserved bit
                RB_LCR_GP_BIT__RB_LCR_DLAB: u1,
            }),
            ///  RO, UART1 interrupt identification
            R8_UART1_IIR: mmio.Mmio(packed struct(u8) {
                ///  RO, UART no interrupt flag: 0=interrupt action, 1=no interrupt
                RB_IIR_NO_INT: u1,
                reserved7: u6,
                ///  RO, UART FIFO enabled flag
                RB_IIR_FIFO_ID: u1,
            }),
            ///  RO, UART1 line status
            R8_UART1_LSR: mmio.Mmio(packed struct(u8) {
                ///  RO, UART receiver fifo data ready status
                RB_LSR_DATA_RDY: u1,
                ///  RZ, UART receiver overrun error
                RB_LSR_OVER_ERR: u1,
                ///  RZ, UART receiver parity error
                RB_LSR_PAR_ERR: u1,
                ///  RZ, UART receiver frame error
                RB_LSR_FRAME_ERR: u1,
                ///  RZ, UART receiver break error
                RB_LSR_BREAK_ERR: u1,
                ///  RO, UART transmitter fifo empty status
                RB_LSR_TX_FIFO_EMP: u1,
                ///  RO, UART transmitter all empty status
                RB_LSR_TX_ALL_EMP: u1,
                ///  RO, indicate error in UART receiver fifo
                RB_LSR_ERR_RX_FIFO: u1,
            }),
            reserved8: [2]u8,
            ///  RO, UART1 receiver buffer, receiving byte
            R8_UART1_RBR: mmio.Mmio(packed struct(u8) {
                ///  RO, UART1 receiver buffer, receiving byte
                R8_UART1_RBR: u8,
            }),
            reserved10: [1]u8,
            ///  RO, UART1 receiver FIFO count
            R8_UART1_RFC: mmio.Mmio(packed struct(u8) {
                ///  RO, UART1 receiver FIFO count
                R8_UART1_RFC: u8,
            }),
            ///  RO, UART1 transmitter FIFO count
            R8_UART1_TFC: mmio.Mmio(packed struct(u8) {
                ///  RO, UART1 receiver FIFO count
                R8_UART1_TFC: u8,
            }),
            ///  RW, UART1 divisor latch
            R16_UART1_DL: mmio.Mmio(packed struct(u16) {
                ///  RW, UART1 divisor latch
                R16_UART1_DL: u16,
            }),
            ///  RW, UART1 pre-divisor latch byte, only low 7 bit, from 1 to 128
            R8_UART1_DIV: mmio.Mmio(packed struct(u8) {
                ///  RW, UART1 pre-divisor latch byte, only low 7 bit, from 1 to 128
                R8_UART1_DIV: u8,
            }),
        };

        ///  UART2 register
        pub const UART2 = extern struct {
            ///  RW, UART2 modem control
            R8_UART2_MCR: mmio.Mmio(packed struct(u8) {
                reserved3: u3,
                ///  RW, UART control OUT2;UART interrupt output enable
                RB_MCR_OUT2__RB_MCR_INT_OE: u1,
                padding: u4,
            }),
            ///  RW, UART2 interrupt enable
            R8_UART2_IER: mmio.Mmio(packed struct(u8) {
                ///  RW, UART interrupt enable for receiver data ready
                RB_IER_RECV_RDY: u1,
                ///  RW, UART interrupt enable for THR empty
                RB_IER_THR_EMPTY: u1,
                ///  RW, UART interrupt enable for receiver line status
                RB_IER_LINE_STAT: u1,
                reserved6: u3,
                ///  RW, UART TXD pin enable
                RB_IER_TXD_EN: u1,
                ///  WZ, UART software reset control, high action, auto clear
                RB_IER_RESET: u1,
            }),
            ///  RW, UART2 FIFO control
            R8_UART2_FCR: mmio.Mmio(packed struct(u8) {
                ///  RW, UART FIFO enable
                RB_FCR_FIFO_EN: u1,
                ///  WZ, clear UART receiver FIFO, high action, auto clear
                RB_FCR_RX_FIFO_CLR: u1,
                ///  WZ, clear UART transmitter FIFO, high action, auto clear
                RB_FCR_TX_FIFO_CLR: u1,
                reserved6: u3,
                ///  RW, UART receiver FIFO trigger level: 00-1byte, 01-2bytes, 10-4bytes, 11-7bytes
                RB_FCR_FIFO_TRIG: u2,
            }),
            ///  RW, UART2 line control
            R8_UART2_LCR: mmio.Mmio(packed struct(u8) {
                ///  RW, UART word bit length: 00-5bit, 01-6bit, 10-7bit, 11-8bit
                RB_LCR_WORD_SZ: u2,
                ///  RW, UART stop bit length: 0-1bit, 1-2bit
                RB_LCR_STOP_BIT: u1,
                ///  RW, UART parity enable
                RB_LCR_PAR_EN: u1,
                ///  RW, UART parity mode: 00-odd, 01-even, 10-mark, 11-space
                RB_LCR_PAR_MOD: u2,
                ///  RW, UART break control enable
                RB_LCR_BREAK_EN: u1,
                ///  RW, UART general purpose bit;RW, UART reserved bit
                RB_LCR_GP_BIT__RB_LCR_DLAB: u1,
            }),
            ///  RO, UART2 interrupt identification
            R8_UART2_IIR: mmio.Mmio(packed struct(u8) {
                ///  RO, UART no interrupt flag: 0=interrupt action, 1=no interrupt
                RB_IIR_NO_INT: u1,
                reserved7: u6,
                ///  RO, UART FIFO enabled flag
                RB_IIR_FIFO_ID: u1,
            }),
            ///  RO, UART2 line status
            R8_UART2_LSR: mmio.Mmio(packed struct(u8) {
                ///  RO, UART receiver fifo data ready status
                RB_LSR_DATA_RDY: u1,
                ///  RZ, UART receiver overrun error
                RB_LSR_OVER_ERR: u1,
                ///  RZ, UART receiver parity error
                RB_LSR_PAR_ERR: u1,
                ///  RZ, UART receiver frame error
                RB_LSR_FRAME_ERR: u1,
                ///  RZ, UART receiver break error
                RB_LSR_BREAK_ERR: u1,
                ///  RO, UART transmitter fifo empty status
                RB_LSR_TX_FIFO_EMP: u1,
                ///  RO, UART transmitter all empty status
                RB_LSR_TX_ALL_EMP: u1,
                ///  RO, indicate error in UART receiver fifo
                RB_LSR_ERR_RX_FIFO: u1,
            }),
            reserved8: [2]u8,
            ///  RO, UART2 receiver buffer, receiving byte
            R8_UART2_RBR: mmio.Mmio(packed struct(u8) {
                ///  RO, UART2 receiver buffer, receiving byte
                R8_UART2_RBR: u8,
            }),
            reserved10: [1]u8,
            ///  RO, UART2 receiver FIFO count
            R8_UART2_RFC: mmio.Mmio(packed struct(u8) {
                ///  RO, UART2 receiver FIFO count
                R8_UART2_RFC: u8,
            }),
            ///  RO, UART2 transmitter FIFO count
            R8_UART2_TFC: mmio.Mmio(packed struct(u8) {
                ///  RO, UART2 transmitter FIFO count
                R8_UART2_TFC: u8,
            }),
            ///  RW, UART2 divisor latch
            R16_UART2_DL: mmio.Mmio(packed struct(u16) {
                ///  RW, UART2 divisor latch
                R16_UART2_DL: u16,
            }),
            ///  RW, UART2 pre-divisor latch byte, only low 7 bit, from 1 to 128
            R8_UART2_DIV: mmio.Mmio(packed struct(u8) {
                ///  RW, UART2 pre-divisor latch byte, only low 7 bit, from 1 to 128
                R8_UART2_DIV: u8,
            }),
        };

        ///  UART3 register
        pub const UART3 = extern struct {
            ///  RW, UART3 modem control
            R8_UART3_MCR: mmio.Mmio(packed struct(u8) {
                reserved3: u3,
                ///  RW, UART control OUT2;UART interrupt output enable
                RB_MCR_OUT2__RB_MCR_INT_OE: u1,
                padding: u4,
            }),
            ///  RW, UART3 interrupt enable
            R8_UART3_IER: mmio.Mmio(packed struct(u8) {
                ///  RW, UART interrupt enable for receiver data ready
                RB_IER_RECV_RDY: u1,
                ///  RW, UART interrupt enable for THR empty
                RB_IER_THR_EMPTY: u1,
                ///  RW, UART interrupt enable for receiver line status
                RB_IER_LINE_STAT: u1,
                reserved6: u3,
                ///  RW, UART TXD pin enable
                RB_IER_TXD_EN: u1,
                ///  WZ, UART software reset control, high action, auto clear
                RB_IER_RESET: u1,
            }),
            ///  RW, UART3 FIFO control
            R8_UART3_FCR: mmio.Mmio(packed struct(u8) {
                ///  RW, UART FIFO enable
                RB_FCR_FIFO_EN: u1,
                ///  WZ, clear UART receiver FIFO, high action, auto clear
                RB_FCR_RX_FIFO_CLR: u1,
                ///  WZ, clear UART transmitter FIFO, high action, auto clear
                RB_FCR_TX_FIFO_CLR: u1,
                reserved6: u3,
                ///  RW, UART receiver FIFO trigger level: 00-1byte, 01-2bytes, 10-4bytes, 11-7bytes
                RB_FCR_FIFO_TRIG: u2,
            }),
            ///  RW, UART3 line control
            R8_UART3_LCR: mmio.Mmio(packed struct(u8) {
                ///  RW, UART word bit length: 00-5bit, 01-6bit, 10-7bit, 11-8bit
                RB_LCR_WORD_SZ: u2,
                ///  RW, UART stop bit length: 0-1bit, 1-2bit
                RB_LCR_STOP_BIT: u1,
                ///  RW, UART parity enable
                RB_LCR_PAR_EN: u1,
                ///  RW, UART parity mode: 00-odd, 01-even, 10-mark, 11-space
                RB_LCR_PAR_MOD: u2,
                ///  RW, UART break control enable
                RB_LCR_BREAK_EN: u1,
                ///  RW, UART general purpose bit;RW, UART reserved bit
                RB_LCR_GP_BIT__RB_LCR_DLAB: u1,
            }),
            ///  RO, UART3 interrupt identification
            R8_UART3_IIR: mmio.Mmio(packed struct(u8) {
                ///  RO, UART no interrupt flag: 0=interrupt action, 1=no interrupt
                RB_IIR_NO_INT: u1,
                ///  RO, UART interrupt flag bit mask
                RB_IIR_INT_MASK: u3,
                reserved7: u3,
                ///  RO, UART FIFO enabled flag
                RB_IIR_FIFO_ID: u1,
            }),
            ///  RO, UART3 line status
            R8_UART3_LSR: mmio.Mmio(packed struct(u8) {
                ///  RO, UART receiver fifo data ready status
                RB_LSR_DATA_RDY: u1,
                ///  RZ, UART receiver overrun error
                RB_LSR_OVER_ERR: u1,
                ///  RZ, UART receiver parity error
                RB_LSR_PAR_ERR: u1,
                ///  RZ, UART receiver frame error
                RB_LSR_FRAME_ERR: u1,
                ///  RZ, UART receiver break error
                RB_LSR_BREAK_ERR: u1,
                ///  RO, UART transmitter fifo empty status
                RB_LSR_TX_FIFO_EMP: u1,
                ///  RO, UART transmitter all empty status
                RB_LSR_TX_ALL_EMP: u1,
                ///  RO, indicate error in UART receiver fifo
                RB_LSR_ERR_RX_FIFO: u1,
            }),
            reserved8: [2]u8,
            ///  RO, UART3 receiver buffer, receiving byte
            R8_UART3_RBR: mmio.Mmio(packed struct(u8) {
                ///  RO, UART3 receiver buffer, receiving byte
                R8_UART3_RBR: u8,
            }),
            reserved10: [1]u8,
            ///  RO, UART3 receiver FIFO count
            R8_UART3_RFC: mmio.Mmio(packed struct(u8) {
                ///  RO, UART3 receiver FIFO count
                R8_UART3_RFC: u8,
            }),
            ///  RO, UART3 transmitter FIFO count
            R8_UART3_TFC: mmio.Mmio(packed struct(u8) {
                ///  RO, UART3 transmitter FIFO count
                R8_UART3_TFC: u8,
            }),
            ///  RW, UART3 divisor latch
            R16_UART3_DL: mmio.Mmio(packed struct(u16) {
                ///  RW, UART3 divisor latch
                R16_UART3_DL: u16,
            }),
            ///  RW, UART3 pre-divisor latch byte, only low 7 bit, from 1 to 128
            R8_UART3_DIV: mmio.Mmio(packed struct(u8) {
                ///  RW, UART3 pre-divisor latch byte, only low 7 bit, from 1 to 128
                R8_UART3_DIV: u8,
            }),
        };

        ///  SPI0 register
        pub const SPI0 = extern struct {
            ///  RW, SPI0 mode control
            R8_SPI0_CTRL_MOD: mmio.Mmio(packed struct(u8) {
                ///  RW, SPI0 slave mode: 0=master or host, 1=slave or device
                RB_SPI_MODE_SLAVE: u1,
                ///  RW, force clear SPI FIFO and count
                RB_SPI_ALL_CLEAR: u1,
                ///  RW, SPI0 enable 2 wire mode for slave: 0=3wire(SCK0,MOSI,MISO), 1=2wire(SCK0,MISO=MXSX)
                RB_SPI_2WIRE_MOD: u1,
                ///  RW, SPI master clock mode: 0=mode 0, 1=mode 3;RW, SPI0 slave command mode: 0=byte stream, 1=first byte command
                RB_SPI_MST_SCK_MOD__RB_SPI_SLV_CMD_MOD: u1,
                ///  RW, SPI FIFO direction: 0=out(write @master mode), 1=in(read @master mode)
                RB_SPI_FIFO_DIR: u1,
                ///  RW, SPI SCK output enable
                RB_SPI_SCK_OE: u1,
                ///  RW, SPI MOSI output enable
                RB_SPI_MOSI_OE: u1,
                ///  RW, SPI MISO output enable
                RB_SPI_MISO_OE: u1,
            }),
            ///  RW, SPI0 configuration control
            R8_SPI0_CTRL_CFG: mmio.Mmio(packed struct(u8) {
                ///  RW, SPI0 DMA enable
                RB_SPI_DMA_ENABLE: u1,
                reserved2: u1,
                ///  RW, SPI0 DMA address loop enable
                RB_SPI_DMA_LOOP: u1,
                reserved4: u1,
                ///  RW, enable buffer/FIFO accessing to auto clear RB_SPI_IF_BYTE_END interrupt flag
                RB_SPI_AUTO_IF: u1,
                ///  RW, SPI bit data order: 0=MSB first, 1=LSB first
                RB_SPI_BIT_ORDER: u1,
                ///  RW, SPI master input delay enable
                RB_SPI_MST_DLY_EN: u1,
                padding: u1,
            }),
            ///  RW, SPI0 interrupt enable
            R8_SPI0_INTER_EN: mmio.Mmio(packed struct(u8) {
                ///  RW, enable interrupt for SPI total byte count end
                RB_SPI_IE_CNT_END: u1,
                ///  RW, enable interrupt for SPI byte exchanged
                RB_SPI_IE_BYTE_END: u1,
                ///  RW, enable interrupt for SPI FIFO half
                RB_SPI_IE_FIFO_HF: u1,
                ///  RW, enable interrupt for SPI0 DMA completion
                RB_SPI_IE_DMA_END: u1,
                ///  RW, enable interrupt for SPI0 FIFO overflow
                RB_SPI_IE_FIFO_OV: u1,
                reserved7: u2,
                ///  RW, enable interrupt for SPI0 slave mode first byte received
                RB_SPI_IE_FST_BYTE: u1,
            }),
            ///  RW, SPI0 master clock divisor;RW, SPI0 slave preset value
            R8_SPI0_CLOCK_DIV__R8_SPI0_SLAVE_PRE: mmio.Mmio(packed struct(u8) {
                ///  RW, SPI0 master clock divisor;RW, SPI0 slave preset value
                R8_SPI0_CLOCK_DIV__R8_SPI0_SLAVE_PRE: u8,
            }),
            ///  RW, SPI0 data buffer
            R8_SPI0_BUFFER: mmio.Mmio(packed struct(u8) {
                ///  RW, SPI0 data buffer
                R8_SPI0_BUFFER: u8,
            }),
            ///  RO, SPI0 work flag
            R8_SPI0_RUN_FLAG: mmio.Mmio(packed struct(u8) {
                reserved4: u4,
                ///  RO, SPI0 slave first byte or command flag
                RB_SPI_SLV_CMD_ACT: u1,
                ///  RO, SPI FIFO ready status
                RB_SPI_FIFO_READY: u1,
                ///  RO, SPI0 slave chip-select loading status
                RB_SPI_SLV_CS_LOAD: u1,
                ///  RO, SPI0 slave selection status
                RB_SPI_SLV_SELECT: u1,
            }),
            ///  RW1, SPI0 interrupt flag
            R8_SPI0_INT_FLAG: mmio.Mmio(packed struct(u8) {
                ///  RW1, interrupt flag for SPI total byte count end
                RB_SPI_IF_CNT_END: u1,
                ///  RW1, interrupt flag for SPI byte exchanged
                RB_SPI_IF_BYTE_END: u1,
                ///  RW1, interrupt flag for SPI FIFO half
                RB_SPI_IF_FIFO_HF: u1,
                ///  RW1, interrupt flag for SPI0 DMA completion
                RB_SPI_IF_DMA_END: u1,
                ///  RW1, interrupt flag for SPI0 FIFO overflow
                RB_SPI_IF_FIFO_OV: u1,
                reserved6: u1,
                ///  RO, current SPI free status
                RB_SPI_FREE: u1,
                ///  RW1, interrupt flag for SPI0 slave mode first byte received
                RB_SPI_IF_FST_BYTE: u1,
            }),
            ///  RO, SPI0 FIFO count status
            R8_SPI0_FIFO_COUNT: mmio.Mmio(packed struct(u8) {
                ///  RO, SPI0 FIFO count status
                R8_SPI0_FIFO_COUNT: u8,
            }),
            reserved12: [4]u8,
            ///  RW, SPI0 total byte count, only low 12 bit
            R16_SPI0_TOTAL_CNT: mmio.Mmio(packed struct(u16) {
                ///  RW, SPI0 total byte count, only low 12 bit
                R16_SPI0_TOTAL_CNT: u16,
            }),
            reserved16: [2]u8,
            ///  RO/WO, SPI0 FIFO register
            R8_SPI0_FIFO: mmio.Mmio(packed struct(u8) {
                ///  RO/WO, SPI0 FIFO register
                R8_SPI0_FIFO: u8,
            }),
            reserved19: [2]u8,
            ///  RO, SPI0 FIFO count status
            R8_SPI0_FIFO_COUNT1: mmio.Mmio(packed struct(u8) {
                ///  RO, SPI0 FIFO count status
                R8_SPI0_FIFO_COUNT1: u8,
            }),
            ///  RW, SPI0 DMA current address
            R16_SPI0_DMA_NOW: mmio.Mmio(packed struct(u16) {
                ///  RW, SPI0 DMA current address
                R16_SPI0_DMA_NOW: u16,
            }),
            reserved24: [2]u8,
            ///  RW, SPI0 DMA begin address
            R16_SPI0_DMA_BEG: mmio.Mmio(packed struct(u16) {
                ///  RW, SPI0 DMA begin address
                R16_SPI0_DMA_BEG: u16,
            }),
            reserved28: [2]u8,
            ///  RW, SPI0 DMA end address
            R16_SPI0_DMA_END: mmio.Mmio(packed struct(u16) {
                ///  RW, SPI0 DMA end address
                R16_SPI0_DMA_END: u16,
            }),
        };

        ///  SPI1 register
        pub const SPI1 = extern struct {
            ///  RW, SPI1 mode control
            R8_SPI1_CTRL_MOD: mmio.Mmio(packed struct(u8) {
                ///  RW, SPI0 slave mode: 0=master or host, 1=slave or device
                RB_SPI_MODE_SLAVE: u1,
                ///  RW, force clear SPI FIFO and count
                RB_SPI_ALL_CLEAR: u1,
                ///  RW, SPI0 enable 2 wire mode for slave: 0=3wire(SCK0,MOSI,MISO), 1=2wire(SCK0,MISO=MXSX)
                RB_SPI_2WIRE_MOD: u1,
                ///  RW, SPI master clock mode: 0=mode 0, 1=mode 3;RW, SPI0 slave command mode: 0=byte stream, 1=first byte command
                RB_SPI_MST_SCK_MOD__RB_SPI_SLV_CMD_MOD: u1,
                ///  RW, SPI FIFO direction: 0=out(write @master mode), 1=in(read @master mode)
                RB_SPI_FIFO_DIR: u1,
                ///  RW, SPI SCK output enable
                RB_SPI_SCK_OE: u1,
                ///  RW, SPI MOSI output enable
                RB_SPI_MOSI_OE: u1,
                ///  RW, SPI MISO output enable
                RB_SPI_MISO_OE: u1,
            }),
            ///  RW, SPI1 configuration control
            R8_SPI1_CTRL_CFG: mmio.Mmio(packed struct(u8) {
                ///  RW, SPI0 DMA enable
                RB_SPI_DMA_ENABLE: u1,
                reserved2: u1,
                ///  RW, SPI0 DMA address loop enable
                RB_SPI_DMA_LOOP: u1,
                reserved4: u1,
                ///  RW, enable buffer/FIFO accessing to auto clear RB_SPI_IF_BYTE_END interrupt flag
                RB_SPI_AUTO_IF: u1,
                ///  RW, SPI bit data order: 0=MSB first, 1=LSB first
                RB_SPI_BIT_ORDER: u1,
                ///  RW, SPI master input delay enable
                RB_SPI_MST_DLY_EN: u1,
                padding: u1,
            }),
            ///  RW, SPI1 interrupt enable
            R8_SPI1_INTER_EN: mmio.Mmio(packed struct(u8) {
                ///  RW, enable interrupt for SPI total byte count end
                RB_SPI_IE_CNT_END: u1,
                ///  RW, enable interrupt for SPI byte exchanged
                RB_SPI_IE_BYTE_END: u1,
                ///  RW, enable interrupt for SPI FIFO half
                RB_SPI_IE_FIFO_HF: u1,
                ///  RW, enable interrupt for SPI0 DMA completion
                RB_SPI_IE_DMA_END: u1,
                ///  RW, enable interrupt for SPI0 FIFO overflow
                RB_SPI_IE_FIFO_OV: u1,
                reserved7: u2,
                ///  RW, enable interrupt for SPI0 slave mode first byte received
                RB_SPI_IE_FST_BYTE: u1,
            }),
            ///  RW, SPI1 master clock divisor;
            R8_SPI1_CLOCK_DIV: mmio.Mmio(packed struct(u8) {
                ///  RW, SPI0 master clock divisor;RW, SPI0 slave preset value
                R8_SPI0_CLOCK_DIV__R8_SPI0_SLAVE_PRE: u8,
            }),
            ///  RW, SPI1 data buffer
            R8_SPI1_BUFFER: mmio.Mmio(packed struct(u8) {
                ///  RW, SPI1 data buffer
                R8_SPI1_BUFFER: u8,
            }),
            ///  RO, SPI1 work flag
            R8_SPI1_RUN_FLAG: mmio.Mmio(packed struct(u8) {
                reserved4: u4,
                ///  RO, SPI0 slave first byte or command flag
                RB_SPI_SLV_CMD_ACT: u1,
                ///  RO, SPI FIFO ready status
                RB_SPI_FIFO_READY: u1,
                ///  RO, SPI0 slave chip-select loading status
                RB_SPI_SLV_CS_LOAD: u1,
                ///  RO, SPI0 slave selection status
                RB_SPI_SLV_SELECT: u1,
            }),
            ///  RW1, SPI1 interrupt flag
            R8_SPI1_INT_FLAG: mmio.Mmio(packed struct(u8) {
                ///  RW1, interrupt flag for SPI total byte count end
                RB_SPI_IF_CNT_END: u1,
                ///  RW1, interrupt flag for SPI byte exchanged
                RB_SPI_IF_BYTE_END: u1,
                ///  RW1, interrupt flag for SPI FIFO half
                RB_SPI_IF_FIFO_HF: u1,
                ///  RW1, interrupt flag for SPI0 DMA completion
                RB_SPI_IF_DMA_END: u1,
                ///  RW1, interrupt flag for SPI0 FIFO overflow
                RB_SPI_IF_FIFO_OV: u1,
                reserved6: u1,
                ///  RO, current SPI free status
                RB_SPI_FREE: u1,
                ///  RW1, interrupt flag for SPI0 slave mode first byte received
                RB_SPI_IF_FST_BYTE: u1,
            }),
            ///  RO, SPI1 FIFO count status
            R8_SPI1_FIFO_COUNT: mmio.Mmio(packed struct(u8) {
                ///  RO, SPI0 FIFO count status
                R8_SPI1_FIFO_COUNT: u8,
            }),
            reserved12: [4]u8,
            ///  RW, SPI1 total byte count, only low 12 bit
            R16_SPI1_TOTAL_CNT: mmio.Mmio(packed struct(u16) {
                ///  RW, SPI1 total byte count, only low 12 bit
                R16_SPI1_TOTAL_CNT: u16,
            }),
            reserved16: [2]u8,
            ///  RO/WO, SPI1 FIFO register
            R8_SPI1_FIFO: mmio.Mmio(packed struct(u8) {
                ///  RO/WO, SPI1 FIFO register
                R8_SPI1_FIFO: u8,
            }),
            reserved19: [2]u8,
            ///  RO, SPI1 FIFO count status
            R8_SPI1_FIFO_COUNT1: mmio.Mmio(packed struct(u8) {
                ///  RO, SPI1 FIFO count status
                R8_SPI1_FIFO_COUNT1: u8,
            }),
        };

        ///  I2C register
        pub const I2C = extern struct {
            ///  RW, I2C control 1
            R16_I2C_CTRL1: mmio.Mmio(packed struct(u16) {
                ///  RW, Peripheral enable
                RB_I2C_PE: u1,
                ///  RW, SMBUS mode: 0=I2C mode, 1=SMBUS mode
                RB_I2C_SMBUS: u1,
                reserved3: u1,
                ///  RW, SMBus type: 0=Device, 1=Host
                RB_I2C_SMBTYPE: u1,
                ///  RW, ARP enable
                RB_I2C_EBARP: u1,
                ///  RW, PEC ebable
                RB_I2C_ENPEC: u1,
                ///  RW, General call enable
                RB_I2C_ENGC: u1,
                ///  RW, Clock stretching disable (Slave mode)
                RB_I2C_NOSTRETCH: u1,
                ///  RW, Start generation: master mode: 0=no start, 1=repeated start; slave mode: 0=no start, 1=start at bus free
                RB_I2C_START: u1,
                ///  RW, Stop generation: master mode: 0=no stop, 1=stop after the current byte transfer or after the current Start condition is sent; slave mode: 0=no stop, 1=Release the SCL and SDA lines after the current byte transfer
                RB_I2C_STOP: u1,
                ///  RW, Acknowledge enable
                RB_I2C_ACK: u1,
                ///  RW, Acknowledge/PEC Position (for data reception)
                RB_I2C_POS: u1,
                ///  RW, Packet error checking: 0=No PEC transfer, 1=PEC transfer (in Tx or Rx mode)
                RB_I2C_PEC: u1,
                ///  RW, SMBus alert: 0=Releases SMBA pin high, 1=Drives SMBA pin low.
                RB_I2C_ALERT: u1,
                reserved15: u1,
                ///  RW, Software reset
                RB_I2C_SWRST: u1,
            }),
            reserved4: [2]u8,
            ///  RW, I2C control 2
            R16_I2C_CTRL2: mmio.Mmio(packed struct(u16) {
                ///  RW, Peripheral clock frequency, The minimum allowed frequency is 2 MHz,the maximum frequency is 36 MHz
                RB_I2C_FREQ: u6,
                reserved8: u2,
                ///  RW, Error interrupt enable
                RB_I2C_ITERREN: u1,
                ///  RW, Event interrupt enable
                RB_I2C_ITEVTEN: u1,
                ///  RW, Buffer interrupt enable
                RB_I2C_ITBUFEN: u1,
                padding: u5,
            }),
            reserved8: [2]u8,
            ///  RW, I2C own address register 1
            R16_I2C_OADDR1: mmio.Mmio(packed struct(u16) {
                ///  RW, bit0 of address in 10-bit addressing mode
                RB_I2C_ADD0: u1,
                ///  RW, bit[7:1] of address
                RB_I2C_ADD7_1: u7,
                ///  RW, bit[9:8] of address in 10-bit addressing mode
                RB_I2C_ADD9_8: u2,
                reserved14: u4,
                ///  RW, Should always be kept at 1
                RB_I2C_MUST1: u1,
                ///  RW, Addressing mode (slave mode): 0=7-bit slave address, 1=10-bit slave address
                RB_I2C_ADDMODE: u1,
            }),
            reserved12: [2]u8,
            ///  RW, I2C own address register 2
            R16_I2C_OADDR2: mmio.Mmio(packed struct(u16) {
                ///  RW, Dual addressing mode enable
                RB_I2C_ENDUAL: u1,
                ///  RW, bit[7:1] of address2
                RB_I2C_ADD2: u7,
                padding: u8,
            }),
            reserved16: [2]u8,
            ///  RW, I2C data register
            R16_I2C_DATAR: mmio.Mmio(packed struct(u16) {
                ///  RW, I2C data register
                R16_I2C_DATAR: u8,
                padding: u8,
            }),
            reserved20: [2]u8,
            ///  R0, I2C stauts register 1
            R16_I2C_STAR1: mmio.Mmio(packed struct(u16) {
                ///  RW0, Start bit flag (Master mode)
                RB_I2C_SB: u1,
                ///  RW0, Address sent (master mode)/matched (slave mode) flag
                RB_I2C_ADDR: u1,
                ///  RO, Byte transfer finished flag
                RB_I2C_BTF: u1,
                ///  RO, 10-bit header sent flag (Master mode)
                RB_I2C_ADD10: u1,
                ///  RO, Stop detection flag (slave mode)
                RB_I2C_STOPF: u1,
                reserved6: u1,
                ///  RO, Data register not empty flag (receivers)
                RB_I2C_RxNE: u1,
                ///  RO, Data register empty flag (transmitters)
                RB_I2C_TxE: u1,
                ///  RW0, Bus error flag
                RB_I2C_BERR: u1,
                ///  RW0, Arbitration lost flag (master mode)
                RB_I2C_ARLO: u1,
                ///  RW0, Acknowledge failure flag
                RB_I2C_AF: u1,
                ///  RW0, Overrun/Underrun flag
                RB_I2C_OVR: u1,
                ///  RW0, PEC Error flag in reception
                RB_I2C_PECERR: u1,
                ///  RW0, Timeout or Tlow error flag
                RB_I2C_TIMEOUT: u1,
                reserved15: u1,
                ///  RW0, SMBus alert flag
                RB_I2C_SMBALERT: u1,
            }),
            reserved24: [2]u8,
            ///  R0, I2C status register 2
            R16_I2C_STAR2: mmio.Mmio(packed struct(u16) {
                ///  RO, Mode statu: 0=Slave mode, 1=Master mode
                RB_I2C_MSL: u1,
                ///  RO, Bus busy flag
                RB_I2C_BUSY: u1,
                ///  RO, Trans flag: 0=data bytes received, 1=data bytes transmitted
                RB_I2C_TRA: u1,
                reserved4: u1,
                ///  RO, General call address (Slave mode) received flag
                RB_I2C_GENCALL: u1,
                ///  RO, SMBus device default address (Slave mode) received flag
                RB_I2C_SMBDEFAULT: u1,
                ///  RO, SMBus host header (Slave mode) received flag
                RB_I2C_SMBHOST: u1,
                ///  RO, Dual flag (Slave mode): 0=Received address matched with OAR1, 1=Received address matched with OAR2
                RB_I2C_DUALF: u1,
                ///  RO, Packet error checking register
                RB_I2C_PECX: u8,
            }),
            reserved28: [2]u8,
            ///  RW, I2C clock control register
            R16_I2C_CKCFGR: mmio.Mmio(packed struct(u16) {
                ///  RW, Controls the SCL clock in Fm/Sm mode (Master mode)
                RB_I2C_CCR: u12,
                reserved14: u2,
                ///  RW, Fm mode duty cycle: 0=L/H=2, 1=L/H=16/9
                RB_I2C_DUTY: u1,
                ///  RW, I2C master mode selection: 0=standard mode, 1=fast mode
                RB_I2C_F_S: u1,
            }),
            reserved32: [2]u8,
            ///  RW, I2C trise register
            R16_I2C_RTR: mmio.Mmio(packed struct(u16) {
                ///  RW, Maximum rise time in Fm/Sm mode (Master mode)
                RB_I2C_TRISE: u6,
                padding: u10,
            }),
        };

        ///  PWMx register
        pub const PWMx = extern struct {
            ///  RW, PWM output enable control
            R8_PWM_OUT_EN: mmio.Mmio(packed struct(u8) {
                ///  RW, PWM4 output enable
                RB_PWM4_OUT_EN: u1,
                ///  RW, PWM5 output enable
                RB_PWM5_OUT_EN: u1,
                ///  RW, PWM6 output enable
                RB_PWM6_OUT_EN: u1,
                ///  RW, PWM7 output enable
                RB_PWM7_OUT_EN: u1,
                ///  RW, PWM8 output enable
                RB_PWM8_OUT_EN: u1,
                ///  RW, PWM9 output enable
                RB_PWM9_OUT_EN: u1,
                ///  RW, PWM10 output enable
                RB_PWM10_OUT_EN: u1,
                ///  RW, PWM11 output enable
                RB_PWM11_OUT_EN: u1,
            }),
            ///  RW, PWM output polarity control
            R8_PWM_POLAR: mmio.Mmio(packed struct(u8) {
                ///  RW, PWM4 output polarity: 0=default low and high action, 1=default high and low action
                RB_PWM4_POLAR: u1,
                ///  RW, PWM5 output polarity: 0=default low and high action, 1=default high and low action
                RB_PWM5_POLAR: u1,
                ///  RW, PWM6 output polarity: 0=default low and high action, 1=default high and low action
                RB_PWM6_POLAR: u1,
                ///  RW, PWM7 output polarity: 0=default low and high action, 1=default high and low action
                RB_PWM7_POLAR: u1,
                ///  RW, PWM8 output polarity: 0=default low and high action, 1=default high and low action
                RB_PWM8_POLAR: u1,
                ///  RW, PWM9 output polarity: 0=default low and high action, 1=default high and low action
                RB_PWM9_POLAR: u1,
                ///  RW, PWM10 output polarity: 0=default low and high action, 1=default high and low action
                RB_PWM10_POLAR: u1,
                ///  RW, PWM11 output polarity: 0=default low and high action, 1=default high and low action
                RB_PWM11_POLAR: u1,
            }),
            ///  RW, PWM configuration
            R8_PWM_CONFIG: mmio.Mmio(packed struct(u8) {
                ///  RW, PWM cycle selection: 0=256;128;64;32 clocks, 1=255;127;63;31 clocks
                RB_PWM_CYCLE_SEL: u1,
                ///  RO, PWM stagger cycle status
                RB_PWM_STAG_ST: u1,
                ///  RW, PWM data width mode: 00=8 bits data, 01=7 bits data, 10=6 bits data, 11=5 bits data
                RB_PWM_CYC_MOD: u2,
                ///  RW, PWM4/5 stagger output enable: 0=independent output, 1=stagger output
                RB_PWM4_5_STAG_EN: u1,
                ///  RW, PWM6/7 stagger output enable: 0=independent output, 1=stagger output
                RB_PWM6_7_STAG_EN: u1,
                ///  RW, PWM8/9 stagger output enable: 0=independent output, 1=stagger output
                RB_PWM8_9_STAG_EN: u1,
                ///  RW, PWM10/11 stagger output enable: 0=independent output, 1=stagger output
                RB_PWM10_11_STAG_EN: u1,
            }),
            ///  RW, PWM clock divisor
            R8_PWM_CLOCK_DIV: mmio.Mmio(packed struct(u8) {
                ///  RW, PWM clock divisor
                R8_PWM_CLOCK_DIV: u8,
            }),
            ///  RW, PWM4 data holding
            R8_PWM4_DATA: mmio.Mmio(packed struct(u8) {
                ///  RW, PWM4 data holding
                R8_PWM4_DATA: u8,
            }),
            ///  RW, PWM5 data holding
            R8_PWM5_DATA: mmio.Mmio(packed struct(u8) {
                ///  RW, PWM5 data holding
                R8_PWM5_DATA: u8,
            }),
            ///  RW, PWM6 data holding
            R8_PWM6_DATA: mmio.Mmio(packed struct(u8) {
                ///  RW, PWM6 data holding
                R8_PWM6_DATA: u8,
            }),
            ///  RW, PWM7 data holding
            R8_PWM7_DATA: mmio.Mmio(packed struct(u8) {
                ///  RW, PWM7 data holding
                R8_PWM7_DATA: u8,
            }),
            ///  RW, PWM8 data holding
            R8_PWM8_DATA: mmio.Mmio(packed struct(u8) {
                ///  RW, PWM8 data holding
                R8_PWM8_DATA: u8,
            }),
            ///  RW, PWM9 data holding
            R8_PWM9_DATA: mmio.Mmio(packed struct(u8) {
                ///  RW, PWM9 data holding
                R8_PWM9_DATA: u8,
            }),
            ///  RW, PWM10 data holding
            R8_PWM10_DATA: mmio.Mmio(packed struct(u8) {
                ///  RW, PWM10 data holding
                R8_PWM10_DATA: u8,
            }),
            ///  RW, PWM11 data holding
            R8_PWM11_DATA: mmio.Mmio(packed struct(u8) {
                ///  RW, PWM11 data holding
                R8_PWM11_DATA: u8,
            }),
            ///  RW, PWM interrupt control
            R8_PWM_INT_CTRL: mmio.Mmio(packed struct(u8) {
                ///  RW, enable interrupt for PWM cycle end
                RB_PWM_IE_CYC: u1,
                ///  RW, select PWM cycle interrupt point
                RB_PWM_CYC_PRE: u1,
                reserved7: u5,
                ///  RW1, interrupt flag for PWM cycle end
                RB_PWM_IF_CYC: u1,
            }),
        };

        ///  USB register
        pub const USB = extern struct {
            ///  USB base control
            R8_USB_CTRL: mmio.Mmio(packed struct(u8) {
                ///  DMA enable and DMA interrupt enable for USB
                RB_UC_DMA_EN: u1,
                ///  force clear FIFO and count of USB
                RB_UC_CLR_ALL: u1,
                ///  force reset USB SIE, need software clear
                RB_UC_RESET_SIE: u1,
                ///  enable automatic responding busy for device mode or automatic pause for host mode during interrupt flag UIF_TRANSFER valid
                RB_UC_INT_BUSY: u1,
                ///  bit mask of USB system control
                MASK_UC_SYS_CTRL: u2,
                ///  enable USB low speed: 0=12Mbps, 1=1.5Mbps
                RB_UC_LOW_SPEED: u1,
                ///  enable USB host mode: 0=device mode, 1=host mode
                RB_UC_HOST_MODE: u1,
            }),
            ///  USB device physical prot control
            R8_UDEV_CTRL__R8_UHOST_CTRL: mmio.Mmio(packed struct(u8) {
                ///  enable USB physical port I-O: 0=disable, 1=enable;enable USB port: 0=disable, 1=enable port, automatic disabled if USB device detached
                RB_UD_PORT_EN__RB_UH_PORT_EN: u1,
                ///  general purpose bit;control USB bus reset: 0=normal, 1=force bus reset
                RB_UD_GP_BIT__RB_UH_BUS_RESET: u1,
                ///  enable USB physical port low speed: 0=full speed, 1=low speed;enable USB port low speed: 0=full speed, 1=low speed
                RB_UD_LOW_SPEED__RB_UH_LOW_SPEED: u1,
                reserved4: u1,
                ///  ReadOnly: indicate current UDM pin level
                RB_UD_DM_PIN__RB_UH_DM_PIN: u1,
                ///  ReadOnly: indicate current UDP pin level
                RB_UD_DP_PIN__RB_UH_DP_PIN: u1,
                reserved7: u1,
                ///  disable USB UDP-UDM pulldown resistance: 0=enable pulldown, 1=disable
                RB_UD_PD_DIS__RB_UH_PD_DIS: u1,
            }),
            ///  USB interrupt enable
            R8_USB_INT_EN: mmio.Mmio(packed struct(u8) {
                ///  enable interrupt for USB bus reset event for USB device mode;enable interrupt for USB device detected event for USB host mode
                RB_UIE_BUS_RST__RB_UIE_DETECT: u1,
                ///  enable interrupt for USB transfer completion
                RB_UIE_TRANSFER: u1,
                ///  enable interrupt for USB suspend or resume event
                RB_UIE_SUSPEND: u1,
                ///  enable interrupt for host SOF timer action for USB host mode
                RB_UIE_HST_SOF: u1,
                ///  enable interrupt for FIFO overflow
                RB_UIE_FIFO_OV: u1,
                reserved6: u1,
                ///  enable interrupt for NAK responded for USB device mode
                RB_UIE_DEV_NAK: u1,
                ///  enable interrupt for SOF received for USB device mode
                RB_UIE_DEV_SOF: u1,
            }),
            ///  USB device address
            R8_USB_DEV_AD: mmio.Mmio(packed struct(u8) {
                ///  bit mask for USB device address
                MASK_USB_ADDR: u7,
                ///  general purpose bit
                RB_UDA_GP_BIT: u1,
            }),
            reserved5: [1]u8,
            ///  USB miscellaneous status
            R8_USB_MIS_ST: mmio.Mmio(packed struct(u8) {
                ///  RO, indicate device attached status on USB host
                RB_UMS_DEV_ATTACH: u1,
                ///  RO, indicate UDM level saved at device attached to USB host
                RB_UMS_DM_LEVEL: u1,
                ///  RO, indicate USB suspend status
                RB_UMS_SUSPEND: u1,
                ///  RO, indicate USB bus reset status
                RB_UMS_BUS_RESET: u1,
                ///  RO, indicate USB receiving FIFO ready status (not empty)
                RB_UMS_R_FIFO_RDY: u1,
                ///  RO, indicate USB SIE free status
                RB_UMS_SIE_FREE: u1,
                ///  RO, indicate host SOF timer action status for USB host
                RB_UMS_SOF_ACT: u1,
                ///  RO, indicate host SOF timer presage status
                RB_UMS_SOF_PRES: u1,
            }),
            ///  USB interrupt flag
            R8_USB_INT_FG: mmio.Mmio(packed struct(u8) {
                ///  RW,bus reset event interrupt flag for USB device mode, direct bit address clear or write 1 to clear;device detected event interrupt flag for USB host mode, direct bit address clear or write 1 to clear
                RB_UIF_BUS_RST__RB_UIF_DETECT: u1,
                ///  RW,USB transfer completion interrupt flag, direct bit address clear or write 1 to clear
                RB_UIF_TRANSFER: u1,
                ///  RW,USB suspend or resume event interrupt flag, direct bit address clear or write 1 to clear
                RB_UIF_SUSPEND: u1,
                ///  RW,host SOF timer interrupt flag for USB host, direct bit address clear or write 1 to clear
                RB_UIF_HST_SOF: u1,
                ///  RW,FIFO overflow interrupt flag for USB, direct bit address clear or write 1 to clear
                RB_UIF_FIFO_OV: u1,
                ///  RO, indicate USB SIE free status
                RB_U_SIE_FREE: u1,
                ///  RO, indicate current USB transfer toggle is OK
                RB_U_TOG_OK: u1,
                ///  RO, indicate current USB transfer is NAK received
                RB_U_IS_NAK: u1,
            }),
            ///  USB interrupt status
            R8_USB_INT_ST: mmio.Mmio(packed struct(u8) {
                ///  RO, bit mask of current transfer handshake response for USB host mode: 0000=no response, time out from device, others=handshake response PID received;RO, bit mask of current transfer endpoint number for USB device mode
                MASK_UIS_H_RES__MASK_UIS_ENDP: u4,
                ///  RO, bit mask of current token PID code received for USB device mode
                MASK_UIS_TOKEN: u2,
                ///  RO, indicate current USB transfer toggle is OK
                RB_UIS_TOG_OK: u1,
                ///  RO, indicate current USB transfer is NAK received for USB device mode
                RB_UIS_SETUP_ACT: u1,
            }),
            ///  USB receiving length
            R8_USB_RX_LEN: mmio.Mmio(packed struct(u8) {
                ///  RO,USB receiving length
                R8_USB_RX_LEN: u8,
            }),
            reserved12: [3]u8,
            ///  endpoint 4/1 mode
            R8_UEP4_1_MOD: mmio.Mmio(packed struct(u8) {
                reserved2: u2,
                ///  enable USB endpoint 4 transmittal (IN)
                RB_UEP4_TX_EN: u1,
                ///  enable USB endpoint 4 receiving (OUT)
                RB_UEP4_RX_EN: u1,
                ///  buffer mode of USB endpoint 1
                RB_UEP1_BUF_MOD: u1,
                reserved6: u1,
                ///  enable USB endpoint 1 transmittal (IN)
                RB_UEP1_TX_EN: u1,
                ///  enable USB endpoint 1 receiving (OUT)
                RB_UEP1_RX_EN: u1,
            }),
            ///  endpoint 2_3 mode;host endpoint mode
            R8_UEP2_3_MOD__R8_UH_EP_MOD: mmio.Mmio(packed struct(u8) {
                ///  buffer mode of USB endpoint 2;buffer mode of USB host IN endpoint
                RB_UEP2_BUF_MOD__RB_UH_EP_RBUF_MOD: u1,
                reserved2: u1,
                ///  enable USB endpoint 2 transmittal (IN)
                RB_UEP2_TX_EN: u1,
                ///  enable USB endpoint 2 receiving (OUT);enable USB host IN endpoint receiving
                RB_UEP2_RX_EN__RB_UH_EP_RX_EN: u1,
                ///  buffer mode of USB endpoint 3;buffer mode of USB host OUT endpoint
                RB_UEP3_BUF_MOD__RB_UH_EP_TBUF_MOD: u1,
                reserved6: u1,
                ///  enable USB endpoint 3 transmittal (IN);enable USB host OUT endpoint transmittal
                RB_UEP3_TX_EN__RB_UH_EP_TX_EN: u1,
                ///  enable USB endpoint 3 receiving (OUT)
                RB_UEP3_RX_EN: u1,
            }),
            ///  endpoint 5/6/7 mode
            R8_UEP567_MOD: mmio.Mmio(packed struct(u8) {
                ///  enable USB endpoint 5 transmittal (IN)
                RB_UEP5_TX_EN: u1,
                ///  enable USB endpoint 5 receiving (OUT)
                RB_UEP5_RX_EN: u1,
                ///  enable USB endpoint 6 transmittal (IN)
                RB_UEP6_TX_EN: u1,
                ///  enable USB endpoint 6 receiving (OUT)
                RB_UEP6_RX_EN: u1,
                ///  enable USB endpoint 7 transmittal (IN)
                RB_UEP7_TX_EN: u1,
                ///  enable USB endpoint 7 receiving (OUT)
                RB_UEP7_RX_EN: u1,
                padding: u2,
            }),
            reserved16: [1]u8,
            ///  endpoint 0 DMA buffer address
            R16_UEP0_DMA: mmio.Mmio(packed struct(u16) {
                ///  RW,endpoint 0 DMA buffer address
                R16_UEP0_DMA: u16,
            }),
            reserved20: [2]u8,
            ///  endpoint 1 DMA buffer address
            R16_UEP1_DMA: mmio.Mmio(packed struct(u16) {
                ///  RW,endpoint 1 DMA buffer address
                R16_UEP1_DMA: u16,
            }),
            reserved24: [2]u8,
            ///  endpoint 2 DMA buffer address;host rx endpoint buffer high address
            R16_UEP2_DMA__R16_UH_RX_DMA: mmio.Mmio(packed struct(u16) {
                ///  RW,endpoint 2 DMA buffer address;host rx endpoint buffer high address
                R16_UEP2_DMA: u16,
            }),
            reserved28: [2]u8,
            ///  endpoint 3 DMA buffer address;host tx endpoint buffer high address
            R16_UEP3_DMA__R16_UH_TX_DMA: mmio.Mmio(packed struct(u16) {
                ///  RW,endpoint 3 DMA buffer address;host rx endpoint buffer high address
                R16_UEP3_DMA: u16,
            }),
            reserved32: [2]u8,
            ///  endpoint 0 transmittal length
            R8_UEP0_T_LEN: mmio.Mmio(packed struct(u8) {
                ///  endpoint 0 transmittal length
                R8_UEP0_T_LEN: u8,
            }),
            reserved34: [1]u8,
            ///  endpoint 0 control
            R8_UEP0_CTRL: mmio.Mmio(packed struct(u8) {
                ///  bit mask of handshake response type for USB endpoint X transmittal (IN)
                MASK_UEP_T_RES: u2,
                ///  bit mask of handshake response type for USB endpoint X receiving (OUT)
                MASK_UEP_R_RES: u2,
                ///  enable automatic toggle after successful transfer completion on endpoint 1_2_3: 0=manual toggle, 1=automatic toggle
                RB_UEP_AUTO_TOG: u1,
                reserved6: u1,
                ///  prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
                RB_UEP_T_TOG: u1,
                ///  expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
                RB_UEP_R_TOG: u1,
            }),
            reserved36: [1]u8,
            ///  endpoint 1 transmittal length
            R8_UEP1_T_LEN: mmio.Mmio(packed struct(u8) {
                ///  endpoint 1 transmittal length
                R8_UEP1_T_LEN: u8,
            }),
            reserved38: [1]u8,
            ///  endpoint 1 control;host aux setup
            R8_UEP1_CTRL__R8_UH_SETUP: mmio.Mmio(packed struct(u8) {
                ///  bit mask of handshake response type for USB endpoint X transmittal (IN)
                MASK_UEP_T_RES: u2,
                ///  bit mask of handshake response type for USB endpoint X receiving (OUT)
                MASK_UEP_R_RES: u2,
                ///  enable automatic toggle after successful transfer completion on endpoint 1_2_3: 0=manual toggle, 1=automatic toggle
                RB_UEP_AUTO_TOG: u1,
                reserved6: u1,
                ///  prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1;USB host automatic SOF enable
                RB_UEP_T_TOG__RB_UH_SOF_EN: u1,
                ///  expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1;RB_UH_PRE_PID_EN;USB host PRE PID enable for low speed device via hub
                RB_UEP_R_TOG__RB_UH_PRE_PID_EN: u1,
            }),
            reserved40: [1]u8,
            ///  endpoint 2 transmittal length;host endpoint and PID
            R8_UEP2_T_LEN_R8_UH_EP_PID: mmio.Mmio(packed struct(u8) {
                ///  bit mask of endpoint number for USB host transfer
                MASK_UH_ENDP: u4,
                ///  bit mask of token PID for USB host transfer
                MASK_UH_TOKEN: u4,
            }),
            reserved42: [1]u8,
            ///  endpoint 2 control;host receiver endpoint control
            R8_UEP2_CTRL_R8_UH_RX_CTRL: mmio.Mmio(packed struct(u8) {
                ///  bit mask of handshake response type for USB endpoint X transmittal (IN)
                MASK_UEP_T_RES: u2,
                ///  bit mask of handshake response type for USB endpoint X receiving (OUT)
                MASK_UH_R_RES: u1,
                reserved4: u1,
                ///  enable automatic toggle after successful transfer completion on endpoint 1_2_3: 0=manual toggle, 1=automatic toggle;enable automatic toggle after successful transfer completion: 0=manual toggle, 1=automatic toggle
                RB_UEP_AUTO_TOG__RB_UH_R_AUTO_TOG: u1,
                reserved6: u1,
                ///  prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
                RB_UEP_T_TOG: u1,
                ///  expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1;expected data toggle flag of host receiving (IN): 0=DATA0, 1=DATA1
                RB_UEP_R_TOG__RB_UH_R_TOG: u1,
            }),
            reserved44: [1]u8,
            ///  endpoint 3 transmittal length;host transmittal endpoint transmittal length
            R8_UEP3_T_LEN__R8_UH_TX_LEN: mmio.Mmio(packed struct(u8) {
                ///  endpoint 1 transmittal length
                R8_UEP3_T_LEN__R8_UH_TX_LEN: u8,
            }),
            reserved46: [1]u8,
            ///  endpoint 3 control;host transmittal endpoint control
            R8_UEP3_CTRL__R8_UH_TX_CTRL: mmio.Mmio(packed struct(u8) {
                ///  bit mask of handshake response type for USB endpoint X transmittal (IN)
                RB_UH_T_RES: u2,
                ///  bit mask of handshake response type for USB endpoint X receiving (OUT)
                MASK_UEP_R_RES: u2,
                ///  enable automatic toggle after successful transfer completion on endpoint 1_2_3: 0=manual toggle, 1=automatic toggle
                RB_UEP_AUTO_TOG_RB_UH_T_AUTO_TOG: u1,
                reserved6: u1,
                ///  prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
                RB_UEP_T_TOG_RB_UH_T_TOG: u1,
                ///  expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
                RB_UEP_R_TOG: u1,
            }),
            reserved48: [1]u8,
            ///  endpoint 4 transmittal length
            R8_UEP4_T_LEN: mmio.Mmio(packed struct(u8) {
                ///  endpoint 4 transmittal length
                R8_UEP4_T_LEN: u8,
            }),
            reserved50: [1]u8,
            ///  endpoint 4 control
            R8_UEP4_CTRL: mmio.Mmio(packed struct(u8) {
                ///  bit mask of handshake response type for USB endpoint X transmittal (IN)
                MASK_UEP_T_RES: u2,
                ///  bit mask of handshake response type for USB endpoint X receiving (OUT)
                MASK_UEP_R_RES: u2,
                ///  enable automatic toggle after successful transfer completion on endpoint 1_2_3: 0=manual toggle, 1=automatic toggle
                RB_UEP_AUTO_TOG: u1,
                reserved6: u1,
                ///  prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
                RB_UEP_T_TOG: u1,
                ///  expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
                RB_UEP_R_TOG: u1,
            }),
            reserved84: [33]u8,
            ///  endpoint 5 DMA buffer address
            R16_UEP5_DMA: mmio.Mmio(packed struct(u16) {
                ///  RW,endpoint 5 DMA buffer address;host rx endpoint buffer high address
                R16_UEP5_DMA: u16,
            }),
            reserved88: [2]u8,
            ///  endpoint 6 DMA buffer address
            R16_UEP6_DMA: mmio.Mmio(packed struct(u16) {
                ///  RW,endpoint 6 DMA buffer address;host rx endpoint buffer high address
                R16_UEP6_DMA: u16,
            }),
            reserved92: [2]u8,
            ///  endpoint 7 DMA buffer address
            R16_UEP7_DMA: mmio.Mmio(packed struct(u16) {
                ///  RW,endpoint 7 DMA buffer address;host rx endpoint buffer high address
                R16_UEP7_DMA: u16,
            }),
            reserved100: [6]u8,
            ///  endpoint 5 transmittal length
            R8_UEP5_T_LEN: mmio.Mmio(packed struct(u8) {
                ///  endpoint 5 transmittal length
                R8_UEP5_T_LEN: u8,
            }),
            reserved102: [1]u8,
            ///  endpoint 5 control
            R8_UEP5_CTRL: mmio.Mmio(packed struct(u8) {
                ///  bit mask of handshake response type for USB endpoint X transmittal (IN)
                MASK_UEP_T_RES: u2,
                ///  bit mask of handshake response type for USB endpoint X receiving (OUT)
                MASK_UEP_R_RES: u2,
                ///  enable automatic toggle after successful transfer completion on endpoint 1_2_3: 0=manual toggle, 1=automatic toggle
                RB_UEP_AUTO_TOG: u1,
                reserved6: u1,
                ///  prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
                RB_UEP_T_TOG: u1,
                ///  expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
                RB_UEP_R_TOG: u1,
            }),
            reserved104: [1]u8,
            ///  endpoint 6 transmittal length
            R8_UEP6_T_LEN: mmio.Mmio(packed struct(u8) {
                ///  endpoint 6 transmittal length
                R8_UEP6_T_LEN: u8,
            }),
            reserved106: [1]u8,
            ///  endpoint 6 control
            R8_UEP6_CTRL: mmio.Mmio(packed struct(u8) {
                ///  bit mask of handshake response type for USB endpoint X transmittal (IN)
                MASK_UEP_T_RES: u2,
                ///  bit mask of handshake response type for USB endpoint X receiving (OUT)
                MASK_UEP_R_RES: u2,
                ///  enable automatic toggle after successful transfer completion on endpoint 1_2_3: 0=manual toggle, 1=automatic toggle
                RB_UEP_AUTO_TOG: u1,
                reserved6: u1,
                ///  prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
                RB_UEP_T_TOG: u1,
                ///  expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
                RB_UEP_R_TOG: u1,
            }),
            reserved108: [1]u8,
            ///  endpoint 7 transmittal length
            R8_UEP7_T_LEN: mmio.Mmio(packed struct(u8) {
                ///  endpoint 7 transmittal length
                R8_UEP7_T_LEN: u8,
            }),
            reserved110: [1]u8,
            ///  endpoint 7 control
            R8_UEP7_CTRL: mmio.Mmio(packed struct(u8) {
                ///  bit mask of handshake response type for USB endpoint X transmittal (IN)
                MASK_UEP_T_RES: u2,
                ///  bit mask of handshake response type for USB endpoint X receiving (OUT)
                MASK_UEP_R_RES: u2,
                ///  enable automatic toggle after successful transfer completion on endpoint 1_2_3: 0=manual toggle, 1=automatic toggle
                RB_UEP_AUTO_TOG: u1,
                reserved6: u1,
                ///  prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
                RB_UEP_T_TOG: u1,
                ///  expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
                RB_UEP_R_TOG: u1,
            }),
        };

        ///  USB2 register
        pub const USB2 = extern struct {
            ///  USB2 base control
            R8_USB2_CTRL: mmio.Mmio(packed struct(u8) {
                ///  DMA enable and DMA interrupt enable for USB
                RB_UC_DMA_EN: u1,
                ///  force clear FIFO and count of USB
                RB_UC_CLR_ALL: u1,
                ///  force reset USB SIE, need software clear
                RB_UC_RESET_SIE: u1,
                ///  enable automatic responding busy for device mode or automatic pause for host mode during interrupt flag UIF_TRANSFER valid
                RB_UC_INT_BUSY: u1,
                ///  bit mask of USB system control
                MASK_UC_SYS_CTRL: u2,
                ///  enable USB low speed: 0=12Mbps, 1=1.5Mbps
                RB_UC_LOW_SPEED: u1,
                ///  enable USB host mode: 0=device mode, 1=host mode
                RB_UC_HOST_MODE: u1,
            }),
            ///  USB2 device physical prot control
            R8_U2DEV_CTRL__R8_U2HOST_CTRL: mmio.Mmio(packed struct(u8) {
                ///  enable USB physical port I-O: 0=disable, 1=enable;enable USB port: 0=disable, 1=enable port, automatic disabled if USB device detached
                RB_UD_PORT_EN__RB_UH_PORT_EN: u1,
                ///  general purpose bit;control USB bus reset: 0=normal, 1=force bus reset
                RB_UD_GP_BIT__RB_UH_BUS_RESET: u1,
                ///  enable USB physical port low speed: 0=full speed, 1=low speed;enable USB port low speed: 0=full speed, 1=low speed
                RB_UD_LOW_SPEED__RB_UH_LOW_SPEED: u1,
                reserved4: u1,
                ///  ReadOnly: indicate current UDM pin level
                RB_UD_DM_PIN__RB_UH_DM_PIN: u1,
                ///  ReadOnly: indicate current UDP pin level
                RB_UD_DP_PIN__RB_UH_DP_PIN: u1,
                reserved7: u1,
                ///  disable USB UDP-UDM pulldown resistance: 0=enable pulldown, 1=disable
                RB_UD_PD_DIS__RB_UH_PD_DIS: u1,
            }),
            ///  USB2 interrupt enable
            R8_USB2_INT_EN: mmio.Mmio(packed struct(u8) {
                ///  enable interrupt for USB bus reset event for USB device mode;enable interrupt for USB device detected event for USB host mode
                RB_UIE_BUS_RST__RB_UIE_DETECT: u1,
                ///  enable interrupt for USB transfer completion
                RB_UIE_TRANSFER: u1,
                ///  enable interrupt for USB suspend or resume event
                RB_UIE_SUSPEND: u1,
                ///  enable interrupt for host SOF timer action for USB host mode
                RB_UIE_HST_SOF: u1,
                ///  enable interrupt for FIFO overflow
                RB_UIE_FIFO_OV: u1,
                reserved6: u1,
                ///  enable interrupt for NAK responded for USB device mode
                RB_UIE_DEV_NAK: u1,
                ///  enable interrupt for SOF received for USB device mode
                RB_UIE_DEV_SOF: u1,
            }),
            ///  USB2 device address
            R8_USB2_DEV_AD: mmio.Mmio(packed struct(u8) {
                ///  bit mask for USB device address
                MASK_USB_ADDR: u7,
                ///  general purpose bit
                RB_UDA_GP_BIT: u1,
            }),
            reserved5: [1]u8,
            ///  USB2 miscellaneous status
            R8_USB2_MIS_ST: mmio.Mmio(packed struct(u8) {
                ///  RO, indicate device attached status on USB host
                RB_UMS_DEV_ATTACH: u1,
                ///  RO, indicate UDM level saved at device attached to USB host
                RB_UMS_DM_LEVEL: u1,
                ///  RO, indicate USB suspend status
                RB_UMS_SUSPEND: u1,
                ///  RO, indicate USB bus reset status
                RB_UMS_BUS_RESET: u1,
                ///  RO, indicate USB receiving FIFO ready status (not empty)
                RB_UMS_R_FIFO_RDY: u1,
                ///  RO, indicate USB SIE free status
                RB_UMS_SIE_FREE: u1,
                ///  RO, indicate host SOF timer action status for USB host
                RB_UMS_SOF_ACT: u1,
                ///  RO, indicate host SOF timer presage status
                RB_UMS_SOF_PRES: u1,
            }),
            ///  USB2 interrupt flag
            R8_USB2_INT_FG: mmio.Mmio(packed struct(u8) {
                ///  RW,bus reset event interrupt flag for USB device mode, direct bit address clear or write 1 to clear;device detected event interrupt flag for USB host mode, direct bit address clear or write 1 to clear
                RB_UIF_BUS_RST__RB_UIF_DETECT: u1,
                ///  RW,USB transfer completion interrupt flag, direct bit address clear or write 1 to clear
                RB_UIF_TRANSFER: u1,
                ///  RW,USB suspend or resume event interrupt flag, direct bit address clear or write 1 to clear
                RB_UIF_SUSPEND: u1,
                ///  RW,host SOF timer interrupt flag for USB host, direct bit address clear or write 1 to clear
                RB_UIF_HST_SOF: u1,
                ///  RW,FIFO overflow interrupt flag for USB, direct bit address clear or write 1 to clear
                RB_UIF_FIFO_OV: u1,
                ///  RO, indicate USB SIE free status
                RB_U_SIE_FREE: u1,
                ///  RO, indicate current USB transfer toggle is OK
                RB_U_TOG_OK: u1,
                ///  RO, indicate current USB transfer is NAK received
                RB_U_IS_NAK: u1,
            }),
            ///  USB2 interrupt status
            R8_USB2_INT_ST: mmio.Mmio(packed struct(u8) {
                ///  RO, bit mask of current transfer handshake response for USB host mode: 0000=no response, time out from device, others=handshake response PID received;RO, bit mask of current transfer endpoint number for USB device mode
                MASK_UIS_H_RES__MASK_UIS_ENDP: u4,
                ///  RO, bit mask of current token PID code received for USB device mode
                MASK_UIS_TOKEN: u2,
                ///  RO, indicate current USB transfer toggle is OK
                RB_UIS_TOG_OK: u1,
                ///  RO, indicate current USB transfer is NAK received for USB device mode
                RB_UIS_SETUP_ACT: u1,
            }),
            ///  USB2 receiving length
            R8_USB2_RX_LEN: mmio.Mmio(packed struct(u8) {
                ///  RO,USB receiving length
                R8_USB_RX_LEN: u8,
            }),
            reserved12: [3]u8,
            ///  endpoint 4/1 mode
            R8_U2EP4_1_MOD: mmio.Mmio(packed struct(u8) {
                reserved2: u2,
                ///  enable USB endpoint 4 transmittal (IN)
                RB_UEP4_TX_EN: u1,
                ///  enable USB endpoint 4 receiving (OUT)
                RB_UEP4_RX_EN: u1,
                ///  buffer mode of USB endpoint 1
                RB_UEP1_BUF_MOD: u1,
                reserved6: u1,
                ///  enable USB endpoint 1 transmittal (IN)
                RB_UEP1_TX_EN: u1,
                ///  enable USB endpoint 1 receiving (OUT)
                RB_UEP1_RX_EN: u1,
            }),
            ///  endpoint 2_3 mode;host endpoint mode
            R8_U2EP2_3_MOD__R8_U2H_EP_MOD: mmio.Mmio(packed struct(u8) {
                ///  buffer mode of USB endpoint 2;buffer mode of USB host IN endpoint
                RB_UEP2_BUF_MOD__RB_UH_EP_RBUF_MOD: u1,
                reserved2: u1,
                ///  enable USB endpoint 2 transmittal (IN)
                RB_UEP2_TX_EN: u1,
                ///  enable USB endpoint 2 receiving (OUT);enable USB host IN endpoint receiving
                RB_UEP2_RX_EN__RB_UH_EP_RX_EN: u1,
                ///  buffer mode of USB endpoint 3;buffer mode of USB host OUT endpoint
                RB_UEP3_BUF_MOD__RB_UH_EP_TBUF_MOD: u1,
                reserved6: u1,
                ///  enable USB endpoint 3 transmittal (IN);enable USB host OUT endpoint transmittal
                RB_UEP3_TX_EN__RB_UH_EP_TX_EN: u1,
                ///  enable USB endpoint 3 receiving (OUT)
                RB_UEP3_RX_EN: u1,
            }),
            ///  USB2 endpoint 5/6/7 mode
            R8_U2EP567_MOD: mmio.Mmio(packed struct(u8) {
                ///  enable USB endpoint 5 transmittal (IN)
                RB_UEP5_TX_EN: u1,
                ///  enable USB endpoint 5 receiving (OUT)
                RB_UEP5_RX_EN: u1,
                ///  enable USB endpoint 6 transmittal (IN)
                RB_UEP6_TX_EN: u1,
                ///  enable USB endpoint 6 receiving (OUT)
                RB_UEP6_RX_EN: u1,
                ///  enable USB endpoint 7 transmittal (IN)
                RB_UEP7_TX_EN: u1,
                ///  enable USB endpoint 7 receiving (OUT)
                RB_UEP7_RX_EN: u1,
                padding: u2,
            }),
            reserved16: [1]u8,
            ///  endpoint 0 DMA buffer address
            R16_U2EP0_DMA: mmio.Mmio(packed struct(u16) {
                ///  RW,endpoint 0 DMA buffer address
                R16_UEP0_DMA: u16,
            }),
            reserved20: [2]u8,
            ///  USB2 endpoint 1 DMA buffer address
            R16_U2EP1_DMA: mmio.Mmio(packed struct(u16) {
                ///  RW,endpoint 1 DMA buffer address
                R16_UEP1_DMA: u16,
            }),
            reserved24: [2]u8,
            ///  USB2 endpoint 2 DMA buffer address;host rx endpoint buffer high address
            R16_U2EP2_DMA__R16_U2H_RX_DMA: mmio.Mmio(packed struct(u16) {
                ///  RW,endpoint 2 DMA buffer address;host rx endpoint buffer high address
                R16_UEP2_DMA: u16,
            }),
            reserved28: [2]u8,
            ///  USB2 endpoint 3 DMA buffer address;host tx endpoint buffer high address
            R16_U2EP3_DMA__R16_U2H_TX_DMA: mmio.Mmio(packed struct(u16) {
                ///  RW,endpoint 3 DMA buffer address;host rx endpoint buffer high address
                R16_UEP3_DMA: u16,
            }),
            reserved32: [2]u8,
            ///  USB2 endpoint 0 transmittal length
            R8_U2EP0_T_LEN: mmio.Mmio(packed struct(u8) {
                ///  endpoint 0 transmittal length
                R8_UEP0_T_LEN: u8,
            }),
            reserved34: [1]u8,
            ///  USB2 endpoint 0 control
            R8_U2EP0_CTRL: mmio.Mmio(packed struct(u8) {
                ///  bit mask of handshake response type for USB endpoint X transmittal (IN)
                MASK_UEP_T_RES: u2,
                ///  bit mask of handshake response type for USB endpoint X receiving (OUT)
                MASK_UEP_R_RES: u2,
                ///  enable automatic toggle after successful transfer completion on endpoint 1_2_3: 0=manual toggle, 1=automatic toggle
                RB_UEP_AUTO_TOG: u1,
                reserved6: u1,
                ///  prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
                RB_UEP_T_TOG: u1,
                ///  expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
                RB_UEP_R_TOG: u1,
            }),
            reserved36: [1]u8,
            ///  USB2 endpoint 1 transmittal length
            R8_U2EP1_T_LEN: mmio.Mmio(packed struct(u8) {
                ///  endpoint 1 transmittal length
                R8_UEP1_T_LEN: u8,
            }),
            reserved38: [1]u8,
            ///  USB2 endpoint 1 control;host aux setup
            R8_U2EP1_CTRL__R8_U2H_SETUP: mmio.Mmio(packed struct(u8) {
                ///  bit mask of handshake response type for USB endpoint X transmittal (IN)
                MASK_UEP_T_RES: u2,
                ///  bit mask of handshake response type for USB endpoint X receiving (OUT)
                MASK_UEP_R_RES: u2,
                ///  enable automatic toggle after successful transfer completion on endpoint 1_2_3: 0=manual toggle, 1=automatic toggle
                RB_UEP_AUTO_TOG: u1,
                reserved6: u1,
                ///  prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1;USB host automatic SOF enable
                RB_UEP_T_TOG__RB_UH_SOF_EN: u1,
                ///  expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1;RB_UH_PRE_PID_EN;USB host PRE PID enable for low speed device via hub
                RB_UEP_R_TOG__RB_UH_PRE_PID_EN: u1,
            }),
            reserved40: [1]u8,
            ///  USB2 endpoint 2 transmittal length;host endpoint and PID
            R8_U2EP2_T_LEN_R8_U2H_EP_PID: mmio.Mmio(packed struct(u8) {
                ///  bit mask of endpoint number for USB host transfer
                MASK_UH_ENDP: u4,
                ///  bit mask of token PID for USB host transfer
                MASK_UH_TOKEN: u4,
            }),
            reserved42: [1]u8,
            ///  USB2 endpoint 2 control;host receiver endpoint control
            R8_U2EP2_CTRL_R8_U2H_RX_CTRL: mmio.Mmio(packed struct(u8) {
                ///  bit mask of handshake response type for USB endpoint X transmittal (IN)
                MASK_UEP_T_RES: u2,
                ///  bit mask of handshake response type for USB endpoint X receiving (OUT)
                MASK_UH_R_RES: u1,
                reserved4: u1,
                ///  enable automatic toggle after successful transfer completion on endpoint 1_2_3: 0=manual toggle, 1=automatic toggle;enable automatic toggle after successful transfer completion: 0=manual toggle, 1=automatic toggle
                RB_UEP_AUTO_TOG__RB_UH_R_AUTO_TOG: u1,
                reserved6: u1,
                ///  prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
                RB_UEP_T_TOG: u1,
                ///  expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1;expected data toggle flag of host receiving (IN): 0=DATA0, 1=DATA1
                RB_UEP_R_TOG__RB_UH_R_TOG: u1,
            }),
            reserved44: [1]u8,
            ///  USB2 endpoint 3 transmittal length;host transmittal endpoint transmittal length
            R8_U2EP3_T_LEN__R8_U2H_TX_LEN: mmio.Mmio(packed struct(u8) {
                ///  endpoint 1 transmittal length
                R8_UEP3_T_LEN__R8_UH_TX_LEN: u8,
            }),
            reserved46: [1]u8,
            ///  USB2 endpoint 3 control;host transmittal endpoint control
            R8_U2EP3_CTRL__R8_U2H_TX_CTRL: mmio.Mmio(packed struct(u8) {
                ///  bit mask of handshake response type for USB endpoint X transmittal (IN)
                RB_UH_T_RES: u2,
                ///  bit mask of handshake response type for USB endpoint X receiving (OUT)
                MASK_UEP_R_RES: u2,
                ///  enable automatic toggle after successful transfer completion on endpoint 1_2_3: 0=manual toggle, 1=automatic toggle
                RB_UEP_AUTO_TOG_RB_UH_T_AUTO_TOG: u1,
                reserved6: u1,
                ///  prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
                RB_UEP_T_TOG_RB_UH_T_TOG: u1,
                ///  expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
                RB_UEP_R_TOG: u1,
            }),
            reserved48: [1]u8,
            ///  USB2 endpoint 4 transmittal length
            R8_U2EP4_T_LEN: mmio.Mmio(packed struct(u8) {
                ///  endpoint 4 transmittal length
                R8_UEP4_T_LEN: u8,
            }),
            reserved50: [1]u8,
            ///  USB2 endpoint 4 control
            R8_U2EP4_CTRL: mmio.Mmio(packed struct(u8) {
                ///  bit mask of handshake response type for USB endpoint X transmittal (IN)
                MASK_UEP_T_RES: u2,
                ///  bit mask of handshake response type for USB endpoint X receiving (OUT)
                MASK_UEP_R_RES: u2,
                ///  enable automatic toggle after successful transfer completion on endpoint 1_2_3: 0=manual toggle, 1=automatic toggle
                RB_UEP_AUTO_TOG: u1,
                reserved6: u1,
                ///  prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
                RB_UEP_T_TOG: u1,
                ///  expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
                RB_UEP_R_TOG: u1,
            }),
            reserved84: [33]u8,
            ///  USB2 endpoint 5 DMA buffer address
            R16_U2EP5_DMA: mmio.Mmio(packed struct(u16) {
                ///  RW,endpoint 5 DMA buffer address;host rx endpoint buffer high address
                R16_UEP5_DMA: u16,
            }),
            reserved88: [2]u8,
            ///  USB2 endpoint 6 DMA buffer address
            R16_U2EP6_DMA: mmio.Mmio(packed struct(u16) {
                ///  RW,endpoint 6 DMA buffer address;host rx endpoint buffer high address
                R16_UEP6_DMA: u16,
            }),
            reserved92: [2]u8,
            ///  USB2 endpoint 7 DMA buffer address
            R16_U2EP7_DMA: mmio.Mmio(packed struct(u16) {
                ///  RW,endpoint 7 DMA buffer address;host rx endpoint buffer high address
                R16_UEP7_DMA: u16,
            }),
            reserved100: [6]u8,
            ///  USB2 endpoint 5 transmittal length
            R8_U2EP5_T_LEN: mmio.Mmio(packed struct(u8) {
                ///  endpoint 5 transmittal length
                R8_UEP5_T_LEN: u8,
            }),
            reserved102: [1]u8,
            ///  USB2 endpoint 5 control
            R8_U2EP5_CTRL: mmio.Mmio(packed struct(u8) {
                ///  bit mask of handshake response type for USB endpoint X transmittal (IN)
                MASK_UEP_T_RES: u2,
                ///  bit mask of handshake response type for USB endpoint X receiving (OUT)
                MASK_UEP_R_RES: u2,
                ///  enable automatic toggle after successful transfer completion on endpoint 1_2_3: 0=manual toggle, 1=automatic toggle
                RB_UEP_AUTO_TOG: u1,
                reserved6: u1,
                ///  prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
                RB_UEP_T_TOG: u1,
                ///  expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
                RB_UEP_R_TOG: u1,
            }),
            reserved104: [1]u8,
            ///  USB2 endpoint 6 transmittal length
            R8_U2EP6_T_LEN: mmio.Mmio(packed struct(u8) {
                ///  endpoint 6 transmittal length
                R8_UEP6_T_LEN: u8,
            }),
            reserved106: [1]u8,
            ///  USB2 endpoint 6 control
            R8_U2EP6_CTRL: mmio.Mmio(packed struct(u8) {
                ///  bit mask of handshake response type for USB endpoint X transmittal (IN)
                MASK_UEP_T_RES: u2,
                ///  bit mask of handshake response type for USB endpoint X receiving (OUT)
                MASK_UEP_R_RES: u2,
                ///  enable automatic toggle after successful transfer completion on endpoint 1_2_3: 0=manual toggle, 1=automatic toggle
                RB_UEP_AUTO_TOG: u1,
                reserved6: u1,
                ///  prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
                RB_UEP_T_TOG: u1,
                ///  expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
                RB_UEP_R_TOG: u1,
            }),
            reserved108: [1]u8,
            ///  USB2 endpoint 7 transmittal length
            R8_U2EP7_T_LEN: mmio.Mmio(packed struct(u8) {
                ///  endpoint 7 transmittal length
                R8_UEP7_T_LEN: u8,
            }),
            reserved110: [1]u8,
            ///  USB2 endpoint 7 control
            R8_U2EP7_CTRL: mmio.Mmio(packed struct(u8) {
                ///  bit mask of handshake response type for USB endpoint X transmittal (IN)
                MASK_UEP_T_RES: u2,
                ///  bit mask of handshake response type for USB endpoint X receiving (OUT)
                MASK_UEP_R_RES: u2,
                ///  enable automatic toggle after successful transfer completion on endpoint 1_2_3: 0=manual toggle, 1=automatic toggle
                RB_UEP_AUTO_TOG: u1,
                reserved6: u1,
                ///  prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
                RB_UEP_T_TOG: u1,
                ///  expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
                RB_UEP_R_TOG: u1,
            }),
        };

        ///  BLE register
        pub const BLE = struct {};

        ///  Program Fast Interrupt Controller
        pub const PFIC = extern struct {
            ///  RO,Interrupt Status Register 1
            R32_PFIC_ISR1: mmio.Mmio(packed struct(u32) {
                reserved12: u12,
                ///  Interrupt ID Status
                INTENSTA: u20,
            }),
            ///  RO,Interrupt Status Register 2
            R32_PFIC_ISR2: mmio.Mmio(packed struct(u32) {
                ///  Interrupt ID Status
                INTENSTA: u4,
                padding: u28,
            }),
            reserved32: [24]u8,
            ///  RO,Interrupt Pending Register 1
            R32_PFIC_IPR1: mmio.Mmio(packed struct(u32) {
                reserved12: u12,
                ///  PENDSTA
                PENDSTA: u20,
            }),
            ///  RO,Interrupt Pending Register 2
            R32_PFIC_IPR2: mmio.Mmio(packed struct(u32) {
                ///  PENDSTA
                PENDSTA: u4,
                padding: u28,
            }),
            reserved64: [24]u8,
            ///  RW,Interrupt Priority Register
            R32_PFIC_ITHRESDR: mmio.Mmio(packed struct(u32) {
                ///  RW,THRESHOLD
                THRESHOLD: u8,
                padding: u24,
            }),
            reserved72: [4]u8,
            ///  Interrupt Config Register
            R32_PFIC_CFGR: mmio.Mmio(packed struct(u32) {
                reserved7: u7,
                ///  WO,RESETSYS
                RESETSYS: u1,
                reserved16: u8,
                ///  WO,KEYCODE
                KEYCODE: u16,
            }),
            ///  Interrupt Global Register
            R32_PFIC_GISR: mmio.Mmio(packed struct(u32) {
                ///  RO,NESTSTA
                NESTSTA: u8,
                ///  RO,GACTSTA
                GACTSTA: u1,
                ///  RO,GPENDSTA
                GPENDSTA: u1,
                padding: u22,
            }),
            ///  RW,Interrupt Fast ID Config Register
            R32_PFIC_IDCFGR: mmio.Mmio(packed struct(u32) {
                ///  RW,FIID0
                FIID0: u8,
                ///  RW,FIID1
                FIID1: u8,
                ///  RW,FIID2
                FIID2: u8,
                ///  RW,FIID3
                FIID3: u8,
            }),
            reserved96: [12]u8,
            ///  Interrupt 0 address Register
            R32_PFIC_FIADDRR0: mmio.Mmio(packed struct(u32) {
                ///  RW,FI0EN
                FI0EN: u1,
                ///  RW,ADDR0
                ADDR0: u31,
            }),
            ///  Interrupt 1 address Register
            R32_PFIC_FIADDRR1: mmio.Mmio(packed struct(u32) {
                ///  RW,FI1EN
                FI1EN: u1,
                ///  RW,ADDR1
                ADDR1: u31,
            }),
            ///  Interrupt 2 address Register
            R32_PFIC_FIADDRR2: mmio.Mmio(packed struct(u32) {
                ///  RW,FI2EN
                FI2EN: u1,
                ///  RW,ADDR2
                ADDR2: u31,
            }),
            ///  Interrupt 3 address Register
            R32_PFIC_FIADDRR3: mmio.Mmio(packed struct(u32) {
                ///  RW,FI3EN
                FI3EN: u1,
                ///  RW,ADDR3
                ADDR3: u31,
            }),
            reserved256: [144]u8,
            ///  Interrupt Setting Register
            R32_PFIC_IENR1: mmio.Mmio(packed struct(u32) {
                reserved12: u12,
                ///  RW1,INTEN
                INTEN: u20,
            }),
            ///  Interrupt Setting Register
            R32_PFIC_IENR2: mmio.Mmio(packed struct(u32) {
                ///  RW1,INTEN
                INTEN: u4,
                padding: u28,
            }),
            reserved384: [120]u8,
            ///  Interrupt Clear Register
            R32_PFIC_IRER1: mmio.Mmio(packed struct(u32) {
                reserved12: u12,
                ///  RW1,INTRESET
                INTRESET: u20,
            }),
            ///  Interrupt Clear Register
            R32_PFIC_IRER2: mmio.Mmio(packed struct(u32) {
                ///  RW1,INTRESET
                INTRESET: u4,
                padding: u28,
            }),
            reserved512: [120]u8,
            ///  Interrupt Pending Register
            R32_PFIC_IPSR1: mmio.Mmio(packed struct(u32) {
                reserved12: u12,
                ///  RW1,PENDSET
                PENDSET: u20,
            }),
            ///  Interrupt Pending Register
            R32_PFIC_IPSR2: mmio.Mmio(packed struct(u32) {
                ///  RW1,PENDSET
                PENDSET: u4,
                padding: u28,
            }),
            reserved640: [120]u8,
            ///  Interrupt Pending Clear Register
            R32_PFIC_IPRR1: mmio.Mmio(packed struct(u32) {
                reserved12: u12,
                ///  RW1,PENDRESET
                PENDRESET: u20,
            }),
            ///  Interrupt Pending Clear Register
            R32_PFIC_IPRR2: mmio.Mmio(packed struct(u32) {
                ///  RW1,PENDRESET
                PENDRESET: u4,
                padding: u28,
            }),
            reserved768: [120]u8,
            ///  Interrupt ACTIVE Register
            R32_PFIC_IACTR1: mmio.Mmio(packed struct(u32) {
                reserved12: u12,
                ///  RW1,IACTS
                IACTS: u20,
            }),
            ///  Interrupt ACTIVE Register
            R32_PFIC_IACTR2: mmio.Mmio(packed struct(u32) {
                ///  RW1,IACTS
                IACTS: u4,
                padding: u28,
            }),
            reserved1024: [248]u8,
            ///  Interrupt Priority configuration Register 0
            R32_PFIC_IPRIOR0: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 0-3
                IPRIOR0: u32,
            }),
            reserved1056: [28]u8,
            ///  Interrupt Priority configuration Register 1
            R32_PFIC_IPRIOR1: mmio.Mmio(packed struct(u32) {
                ///  >RW,Interrupt priority for number 4-7
                IPRIOR1: u32,
            }),
            reserved1088: [28]u8,
            ///  Interrupt Priority configuration Register 2
            R32_PFIC_IPRIOR2: mmio.Mmio(packed struct(u32) {
                ///  >RW,Interrupt priority for number 8-11
                IPRIOR2: u32,
            }),
            reserved1120: [28]u8,
            ///  Interrupt Priority configuration Register 3
            R32_PFIC_IPRIOR3: mmio.Mmio(packed struct(u32) {
                ///  >RW,Interrupt priority for number 12-15
                IPRIOR3: u32,
            }),
            reserved1152: [28]u8,
            ///  Interrupt Priority configuration Register 4
            R32_PFIC_IPRIOR4: mmio.Mmio(packed struct(u32) {
                ///  >RW,Interrupt priority for number 16-19
                IPRIOR4: u32,
            }),
            reserved1184: [28]u8,
            ///  Interrupt Priority configuration Register 5
            R32_PFIC_IPRIOR5: mmio.Mmio(packed struct(u32) {
                ///  >RW,Interrupt priority for number 20-23
                IPRIOR5: u32,
            }),
            reserved1216: [28]u8,
            ///  Interrupt Priority configuration Register 6
            R32_PFIC_IPRIOR6: mmio.Mmio(packed struct(u32) {
                ///  >RW,Interrupt priority for number 24-27
                IPRIOR6: u32,
            }),
            reserved1248: [28]u8,
            ///  Interrupt Priority configuration Register 7
            R32_PFIC_IPRIOR7: mmio.Mmio(packed struct(u32) {
                ///  >RW,Interrupt priority for number 28-31
                IPRIOR7: u32,
            }),
            reserved1280: [28]u8,
            ///  Interrupt Priority configuration Register 8
            R32_PFIC_IPRIOR8: mmio.Mmio(packed struct(u32) {
                ///  >RW,Interrupt priority for number 32-35
                IPRIOR8: u32,
            }),
            reserved1312: [28]u8,
            ///  Interrupt Priority configuration Register 9
            R32_PFIC_IPRIOR9: mmio.Mmio(packed struct(u32) {
                ///  >RW,Interrupt priority for number 36-39
                IPRIOR9: u32,
            }),
            reserved1344: [28]u8,
            ///  Interrupt Priority configuration Register 10
            R32_PFIC_IPRIOR10: mmio.Mmio(packed struct(u32) {
                ///  >RW,Interrupt priority for number 40-43
                IPRIOR10: u32,
            }),
            reserved1376: [28]u8,
            ///  Interrupt Priority configuration Register 11
            R32_PFIC_IPRIOR11: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 44-47
                IPRIOR11: u32,
            }),
            reserved1408: [28]u8,
            ///  Interrupt Priority configuration Register 12
            R32_PFIC_IPRIOR12: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 48-51
                IPRIOR12: u32,
            }),
            reserved1440: [28]u8,
            ///  Interrupt Priority configuration Register 13
            R32_PFIC_IPRIOR13: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 52-55
                IPRIOR13: u32,
            }),
            reserved1472: [28]u8,
            ///  Interrupt Priority configuration Register 14
            R32_PFIC_IPRIOR14: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 56-59
                IPRIOR14: u32,
            }),
            reserved1504: [28]u8,
            ///  Interrupt Priority configuration Register 15
            R32_PFIC_IPRIOR15: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 60-63
                IPRIOR15: u32,
            }),
            reserved1536: [28]u8,
            ///  Interrupt Priority configuration Register 16
            R32_PFIC_IPRIOR16: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 64-67
                IPRIOR16: u32,
            }),
            reserved1568: [28]u8,
            ///  Interrupt Priority configuration Register 17
            R32_PFIC_IPRIOR17: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 68-71
                IPRIOR17: u32,
            }),
            reserved1600: [28]u8,
            ///  Interrupt Priority configuration Register 18
            R32_PFIC_IPRIOR18: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 72-75
                IPRIOR18: u32,
            }),
            reserved1632: [28]u8,
            ///  Interrupt Priority configuration Register 19
            R32_PFIC_IPRIOR19: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 76-79
                IPRIOR19: u32,
            }),
            reserved1664: [28]u8,
            ///  Interrupt Priority configuration Register 20
            R32_PFIC_IPRIOR20: mmio.Mmio(packed struct(u32) {
                ///  RW,RW,Interrupt priority for number 80-83
                IPRIOR20: u32,
            }),
            reserved1696: [28]u8,
            ///  Interrupt Priority configuration Register 21
            R32_PFIC_IPRIOR21: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 84-87
                IPRIOR21: u32,
            }),
            reserved1728: [28]u8,
            ///  Interrupt Priority configuration Register 22
            R32_PFIC_IPRIOR22: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 88-91
                IPRIOR22: u32,
            }),
            reserved1760: [28]u8,
            ///  Interrupt Priority configuration Register 23
            R32_PFIC_IPRIOR23: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 92-95
                IPRIOR23: u32,
            }),
            reserved1792: [28]u8,
            ///  Interrupt Priority configuration Register 24
            R32_PFIC_IPRIOR24: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 96-99
                IPRIOR24: u32,
            }),
            reserved1824: [28]u8,
            ///  Interrupt Priority configuration Register 25
            R32_PFIC_IPRIOR25: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 100-103
                IPRIOR25: u32,
            }),
            reserved1856: [28]u8,
            ///  Interrupt Priority configuration Register 26
            R32_PFIC_IPRIOR26: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 104-107
                IPRIOR26: u32,
            }),
            reserved1888: [28]u8,
            ///  Interrupt Priority configuration Register 27
            R32_PFIC_IPRIOR27: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 108-111
                IPRIOR27: u32,
            }),
            reserved1920: [28]u8,
            ///  Interrupt Priority configuration Register 28
            R32_PFIC_IPRIOR28: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 112-115
                IPRIOR28: u32,
            }),
            reserved1952: [28]u8,
            ///  Interrupt Priority configuration Register 29
            R32_PFIC_IPRIOR29: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 116-119
                IPRIOR29: u32,
            }),
            reserved1984: [28]u8,
            ///  Interrupt Priority configuration Register 30
            R32_PFIC_IPRIOR30: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 120-123
                IPRIOR30: u32,
            }),
            reserved2016: [28]u8,
            ///  Interrupt Priority configuration Register 31
            R32_PFIC_IPRIOR31: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 124-127
                IPRIOR31: u32,
            }),
            reserved2048: [28]u8,
            ///  Interrupt Priority configuration Register 32
            R32_PFIC_IPRIOR32: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 128-131
                IPRIOR32: u32,
            }),
            reserved2080: [28]u8,
            ///  Interrupt Priority configuration Register 33
            R32_PFIC_IPRIOR33: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 132-135
                IPRIOR33: u32,
            }),
            reserved2112: [28]u8,
            ///  Interrupt Priority configuration Register 34
            R32_PFIC_IPRIOR34: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 136-139
                IPRIOR34: u32,
            }),
            reserved2144: [28]u8,
            ///  Interrupt Priority configuration Register 35
            R32_PFIC_IPRIOR35: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 140-143
                IPRIOR35: u32,
            }),
            reserved2176: [28]u8,
            ///  Interrupt Priority configuration Register 36
            R32_PFIC_IPRIOR36: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 144-147
                IPRIOR36: u32,
            }),
            reserved2208: [28]u8,
            ///  Interrupt Priority configuration Register 37
            R32_PFIC_IPRIOR37: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 148-151
                IPRIOR37: u32,
            }),
            reserved2240: [28]u8,
            ///  Interrupt Priority configuration Register 38
            R32_PFIC_IPRIOR38: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 152-155
                IPRIOR38: u32,
            }),
            reserved2272: [28]u8,
            ///  Interrupt Priority configuration Register 39
            R32_PFIC_IPRIOR39: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 156-159
                IPRIOR39: u32,
            }),
            reserved2304: [28]u8,
            ///  Interrupt Priority configuration Register 40
            R32_PFIC_IPRIOR40: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 160-163
                IPRIOR40: u32,
            }),
            reserved2336: [28]u8,
            ///  Interrupt Priority configuration Register 41
            R32_PFIC_IPRIOR41: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 164-167
                IPRIOR41: u32,
            }),
            reserved2368: [28]u8,
            ///  Interrupt Priority configuration Register 42
            R32_PFIC_IPRIOR42: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 168-171
                IPRIOR42: u32,
            }),
            reserved2400: [28]u8,
            ///  Interrupt Priority configuration Register 43
            R32_PFIC_IPRIOR43: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 172-175
                IPRIOR43: u32,
            }),
            reserved2432: [28]u8,
            ///  Interrupt Priority configuration Register 44
            R32_PFIC_IPRIOR44: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 176-179
                IPRIOR44: u32,
            }),
            reserved2464: [28]u8,
            ///  Interrupt Priority configuration Register 45
            R32_PFIC_IPRIOR45: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 180-183
                IPRIOR45: u32,
            }),
            reserved2496: [28]u8,
            ///  Interrupt Priority configuration Register 46
            R32_PFIC_IPRIOR46: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 184-187
                IPRIOR46: u32,
            }),
            reserved2528: [28]u8,
            ///  Interrupt Priority configuration Register 47
            R32_PFIC_IPRIOR47: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 188-191
                IPRIOR47: u32,
            }),
            reserved2560: [28]u8,
            ///  Interrupt Priority configuration Register 48
            R32_PFIC_IPRIOR48: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 192-195
                IPRIOR48: u32,
            }),
            reserved2592: [28]u8,
            ///  Interrupt Priority configuration Register 49
            R32_PFIC_IPRIOR49: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 196-199
                IPRIOR49: u32,
            }),
            reserved2624: [28]u8,
            ///  Interrupt Priority configuration Register 50
            R32_PFIC_IPRIOR50: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 200-203
                IPRIOR50: u32,
            }),
            reserved2656: [28]u8,
            ///  Interrupt Priority configuration Register 51
            R32_PFIC_IPRIOR51: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 204-207
                IPRIOR51: u32,
            }),
            reserved2688: [28]u8,
            ///  Interrupt Priority configuration Register 52
            R32_PFIC_IPRIOR52: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 208-211
                IPRIOR52: u32,
            }),
            reserved2720: [28]u8,
            ///  Interrupt Priority configuration Register 53
            R32_PFIC_IPRIOR53: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 212-215
                IPRIOR53: u32,
            }),
            reserved2768: [44]u8,
            ///  Interrupt Priority configuration Register 54
            R32_PFIC_IPRIOR54: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 216-219
                IPRIOR54: u32,
            }),
            reserved2784: [12]u8,
            ///  Interrupt Priority configuration Register 55
            R32_PFIC_IPRIOR55: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 220-223
                IPRIOR55: u32,
            }),
            reserved2816: [28]u8,
            ///  Interrupt Priority configuration Register 56
            R32_PFIC_IPRIOR56: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 224-227
                IPRIOR56: u32,
            }),
            reserved2848: [28]u8,
            ///  Interrupt Priority configuration Register 57
            R32_PFIC_IPRIOR57: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 228-231
                IPRIOR57: u32,
            }),
            reserved2880: [28]u8,
            ///  Interrupt Priority configuration Register 58
            R32_PFIC_IPRIOR58: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 232-235
                IPRIOR58: u32,
            }),
            reserved2912: [28]u8,
            ///  Interrupt Priority configuration Register 59
            R32_PFIC_IPRIOR59: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 236-239
                IPRIOR59: u32,
            }),
            reserved2944: [28]u8,
            ///  Interrupt Priority configuration Register 60
            R32_PFIC_IPRIOR60: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 240-243
                IPRIOR60: u32,
            }),
            reserved2976: [28]u8,
            ///  Interrupt Priority configuration Register 61
            R32_PFIC_IPRIOR61: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 244-247
                IPRIOR61: u32,
            }),
            reserved3040: [60]u8,
            ///  Interrupt Priority configuration Register 62
            R32_PFIC_IPRIOR62: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 248-251
                IPRIOR62: u32,
            }),
            reserved3072: [28]u8,
            ///  Interrupt Priority configuration Register 63
            R32_PFIC_IPRIOR63: mmio.Mmio(packed struct(u32) {
                ///  RW,Interrupt priority for number 252-255
                IPRIOR63: u32,
            }),
            reserved3344: [268]u8,
            ///  System Control Register
            R32_PFIC_SCTLR: mmio.Mmio(packed struct(u32) {
                reserved1: u1,
                ///  RW,SLEEPONEXIT
                SLEEPONEXIT: u1,
                ///  RW,SLEEPDEEP
                SLEEPDEEP: u1,
                ///  RW,WFITOWFE
                WFITOWFE: u1,
                ///  RW,SEVONPEND
                SEVONPEND: u1,
                ///  WO,SETEVENT
                SETEVENT: u1,
                padding: u26,
            }),
        };

        ///  Systick register
        pub const Systick = extern struct {
            ///  Systick counter control register
            R32_STK_CTLR: mmio.Mmio(packed struct(u32) {
                ///  Systick counter enable
                STE: u1,
                ///  Systick counter interrupt enable
                STIE: u1,
                ///  System counter clock Source selection
                STCLK: u1,
                ///  System counter reload control
                STRE: u1,
                ///  counter mode
                MODE: u1,
                ///  Initial counter value updated
                INIT: u1,
                reserved31: u25,
                ///  RW0,System soft interrupt enable
                SWIE: u1,
            }),
            ///  Systick count status register
            R32_STK_SR: mmio.Mmio(packed struct(u32) {
                ///  RW,CNTIF
                CNTIF: u1,
                padding: u31,
            }),
            ///  Systick counter low register
            R32_STK_CNTL: mmio.Mmio(packed struct(u32) {
                ///  RW,CNTL
                CNTL: u32,
            }),
            ///  Systick counter high register
            R32_STK_CNTH: mmio.Mmio(packed struct(u32) {
                ///  RW,CNTH
                CNTH: u32,
            }),
            ///  Systick compare low register
            R32_STK_CMPLR: mmio.Mmio(packed struct(u32) {
                ///  RW,CMPL
                CMPL: u32,
            }),
            ///  Systick compare high register
            R32_STK_CMPHR: mmio.Mmio(packed struct(u32) {
                ///  RW,CMPH
                CMPH: u32,
            }),
        };
    };
};
