-----------------------------------------------------------------------
-- @file:   baskara_g1_top.vhd
-- @author: Lucas Manco
-- @date:   21 / 10 / 2017
-- @brief:  Top of the fisrt group of baskara equation
----------------------------------------------------------------------

-- IEEE Libraries
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity baskara_g1_top is
port(
    fpga_clk1_50        : in  std_logic;
    --
    HPS_ENET_GTX_CLK    : out std_logic; 
    HPS_ENET_TX_DATA    : out std_logic_vector(3 downto 0);
    HPS_ENET_MDIO       : inout std_logic;
    HPS_ENET_MDC        : out std_logic;
    HPS_ENET_RX_DV      : in  std_logic;
    HPS_ENET_TX_EN      : out std_logic;
    HPS_ENET_RX_CLK     : in  std_logic;
    HPS_ENET_RX_DATA    : in  std_logic_vector(3 downto 0);
    HPS_ENET_INT_N      : inout std_logic;
    --
    HPS_SD_CMD          : inout std_logic;
    HPS_SD_DATA         : inout std_logic_vector(3 downto 0);
    HPS_SD_CLK          : out std_logic;
    --
    HPS_USB_DATA        : inout std_logic_vector(7 downto 0);
    HPS_USB_CLKOUT      : in  std_logic;
    HPS_USB_STP         : out std_logic;
    HPS_USB_DIR         : in  std_logic;
    HPS_USB_NXT         : in  std_logic;
    --
    HPS_SPIM_CLK        : out std_logic;
    HPS_SPIM_MOSI       : out std_logic;
    HPS_SPIM_MISO       : in  std_logic;
    HPS_SPIM_SS         : out std_logic;
    --
    HPS_UART_RX         : in  std_logic;
    HPS_UART_TX         : out std_logic;
    --
    HPS_I2C0_SDAT       : inout std_logic;
    HPS_I2C0_SCLK       : inout std_logic;
    HPS_I2C1_SDAT       : inout std_logic;
    HPS_I2C1_SCLK       : inout std_logic;
    --
    HPS_CONV_USB_N      : inout std_logic; 
    HPS_LTC_GPIO        : inout std_logic;
    HPS_LED             : inout std_logic;
    HPS_KEY             : inout std_logic;
    HPS_GSENSOR_INT     : inout std_logic;
    --
    HPS_DDR3_ADDR       : out   std_logic_vector(14 downto 0);
    HPS_DDR3_BA         : out   std_logic_vector(2 downto 0);
    HPS_DDR3_CK_P       : out   std_logic;
    HPS_DDR3_CK_N       : out   std_logic;
    HPS_DDR3_CKE        : out   std_logic;
    HPS_DDR3_CS_N       : out   std_logic;
    HPS_DDR3_RAS_N      : out   std_logic;
    HPS_DDR3_CAS_N      : out   std_logic;
    HPS_DDR3_WE_N       : out   std_logic;
    HPS_DDR3_RESET_N    : out   std_logic;
    HPS_DDR3_DQ         : inout std_logic_vector(31 downto 0);
    HPS_DDR3_DQS_P      : inout std_logic_vector(3 downto 0);
    HPS_DDR3_DQS_N      : inout std_logic_vector(3 downto 0);
    HPS_DDR3_ODT        : out   std_logic;
    HPS_DDR3_DM         : out   std_logic_vector(3 downto 0);
    HPS_DDR3_RZQ        : in    std_logic;
    
    
    --
    key_i               : in  std_logic_vector(1 downto 0);
    sw_i                : in  std_logic_vector(3 downto 0);
    led_o               : out std_logic_vector(7 downto 0));
end entity;

architecture behavioral of baskara_g1_top is

    component soc_system is
    port (
        clk_clk                               : in    std_logic                      := 'X';             -- clk
        hps_0_f2h_cold_reset_req_reset_n      : in    std_logic                      := 'X';             -- reset_n
        hps_0_f2h_debug_reset_req_reset_n     : in    std_logic                      := 'X';             -- reset_n
        hps_0_f2h_stm_hw_events_stm_hwevents  : in    std_logic_vector(27 downto 0)  := (others => 'X'); -- stm_hwevents
        hps_0_f2h_warm_reset_req_reset_n      : in    std_logic                      := 'X';             -- reset_n
        hps_0_h2f_reset_reset_n               : out   std_logic;                                         -- reset_n
        hps_0_hps_io_hps_io_emac1_inst_TX_CLK : out   std_logic;                                         -- hps_io_emac1_inst_TX_CLK
        hps_0_hps_io_hps_io_emac1_inst_TXD0   : out   std_logic;                                         -- hps_io_emac1_inst_TXD0
        hps_0_hps_io_hps_io_emac1_inst_TXD1   : out   std_logic;                                         -- hps_io_emac1_inst_TXD1
        hps_0_hps_io_hps_io_emac1_inst_TXD2   : out   std_logic;                                         -- hps_io_emac1_inst_TXD2
        hps_0_hps_io_hps_io_emac1_inst_TXD3   : out   std_logic;                                         -- hps_io_emac1_inst_TXD3
        hps_0_hps_io_hps_io_emac1_inst_RXD0   : in    std_logic                      := 'X';             -- hps_io_emac1_inst_RXD0
        hps_0_hps_io_hps_io_emac1_inst_MDIO   : inout std_logic                      := 'X';             -- hps_io_emac1_inst_MDIO
        hps_0_hps_io_hps_io_emac1_inst_MDC    : out   std_logic;                                         -- hps_io_emac1_inst_MDC
        hps_0_hps_io_hps_io_emac1_inst_RX_CTL : in    std_logic                      := 'X';             -- hps_io_emac1_inst_RX_CTL
        hps_0_hps_io_hps_io_emac1_inst_TX_CTL : out   std_logic;                                         -- hps_io_emac1_inst_TX_CTL
        hps_0_hps_io_hps_io_emac1_inst_RX_CLK : in    std_logic                      := 'X';             -- hps_io_emac1_inst_RX_CLK
        hps_0_hps_io_hps_io_emac1_inst_RXD1   : in    std_logic                      := 'X';             -- hps_io_emac1_inst_RXD1
        hps_0_hps_io_hps_io_emac1_inst_RXD2   : in    std_logic                      := 'X';             -- hps_io_emac1_inst_RXD2
        hps_0_hps_io_hps_io_emac1_inst_RXD3   : in    std_logic                      := 'X';             -- hps_io_emac1_inst_RXD3
        hps_0_hps_io_hps_io_sdio_inst_CMD     : inout std_logic                      := 'X';             -- hps_io_sdio_inst_CMD
        hps_0_hps_io_hps_io_sdio_inst_D0      : inout std_logic                      := 'X';             -- hps_io_sdio_inst_D0
        hps_0_hps_io_hps_io_sdio_inst_D1      : inout std_logic                      := 'X';             -- hps_io_sdio_inst_D1
        hps_0_hps_io_hps_io_sdio_inst_CLK     : out   std_logic;                                         -- hps_io_sdio_inst_CLK
        hps_0_hps_io_hps_io_sdio_inst_D2      : inout std_logic                      := 'X';             -- hps_io_sdio_inst_D2
        hps_0_hps_io_hps_io_sdio_inst_D3      : inout std_logic                      := 'X';             -- hps_io_sdio_inst_D3
        hps_0_hps_io_hps_io_usb1_inst_D0      : inout std_logic                      := 'X';             -- hps_io_usb1_inst_D0
        hps_0_hps_io_hps_io_usb1_inst_D1      : inout std_logic                      := 'X';             -- hps_io_usb1_inst_D1
        hps_0_hps_io_hps_io_usb1_inst_D2      : inout std_logic                      := 'X';             -- hps_io_usb1_inst_D2
        hps_0_hps_io_hps_io_usb1_inst_D3      : inout std_logic                      := 'X';             -- hps_io_usb1_inst_D3
        hps_0_hps_io_hps_io_usb1_inst_D4      : inout std_logic                      := 'X';             -- hps_io_usb1_inst_D4
        hps_0_hps_io_hps_io_usb1_inst_D5      : inout std_logic                      := 'X';             -- hps_io_usb1_inst_D5
        hps_0_hps_io_hps_io_usb1_inst_D6      : inout std_logic                      := 'X';             -- hps_io_usb1_inst_D6
        hps_0_hps_io_hps_io_usb1_inst_D7      : inout std_logic                      := 'X';             -- hps_io_usb1_inst_D7
        hps_0_hps_io_hps_io_usb1_inst_CLK     : in    std_logic                      := 'X';             -- hps_io_usb1_inst_CLK
        hps_0_hps_io_hps_io_usb1_inst_STP     : out   std_logic;                                         -- hps_io_usb1_inst_STP
        hps_0_hps_io_hps_io_usb1_inst_DIR     : in    std_logic                      := 'X';             -- hps_io_usb1_inst_DIR
        hps_0_hps_io_hps_io_usb1_inst_NXT     : in    std_logic                      := 'X';             -- hps_io_usb1_inst_NXT
        hps_0_hps_io_hps_io_spim1_inst_CLK    : out   std_logic;                                         -- hps_io_spim1_inst_CLK
        hps_0_hps_io_hps_io_spim1_inst_MOSI   : out   std_logic;                                         -- hps_io_spim1_inst_MOSI
        hps_0_hps_io_hps_io_spim1_inst_MISO   : in    std_logic                      := 'X';             -- hps_io_spim1_inst_MISO
        hps_0_hps_io_hps_io_spim1_inst_SS0    : out   std_logic;                                         -- hps_io_spim1_inst_SS0
        hps_0_hps_io_hps_io_uart0_inst_RX     : in    std_logic                      := 'X';             -- hps_io_uart0_inst_RX
        hps_0_hps_io_hps_io_uart0_inst_TX     : out   std_logic;                                         -- hps_io_uart0_inst_TX
        hps_0_hps_io_hps_io_i2c0_inst_SDA     : inout std_logic                      := 'X';             -- hps_io_i2c0_inst_SDA
        hps_0_hps_io_hps_io_i2c0_inst_SCL     : inout std_logic                      := 'X';             -- hps_io_i2c0_inst_SCL
        hps_0_hps_io_hps_io_i2c1_inst_SDA     : inout std_logic                      := 'X';             -- hps_io_i2c1_inst_SDA
        hps_0_hps_io_hps_io_i2c1_inst_SCL     : inout std_logic                      := 'X';             -- hps_io_i2c1_inst_SCL
        hps_0_hps_io_hps_io_gpio_inst_GPIO09  : inout std_logic                      := 'X';             -- hps_io_gpio_inst_GPIO09
        hps_0_hps_io_hps_io_gpio_inst_GPIO35  : inout std_logic                      := 'X';             -- hps_io_gpio_inst_GPIO35
        hps_0_hps_io_hps_io_gpio_inst_GPIO40  : inout std_logic                      := 'X';             -- hps_io_gpio_inst_GPIO40
        hps_0_hps_io_hps_io_gpio_inst_GPIO53  : inout std_logic                      := 'X';             -- hps_io_gpio_inst_GPIO53
        hps_0_hps_io_hps_io_gpio_inst_GPIO54  : inout std_logic                      := 'X';             -- hps_io_gpio_inst_GPIO54
        hps_0_hps_io_hps_io_gpio_inst_GPIO61  : inout std_logic                      := 'X';             -- hps_io_gpio_inst_GPIO61
        memory_mem_a                          : out   std_logic_vector(14 downto 0);                     -- mem_a
        memory_mem_ba                         : out   std_logic_vector(2 downto 0);                      -- mem_ba
        memory_mem_ck                         : out   std_logic;                                         -- mem_ck
        memory_mem_ck_n                       : out   std_logic;                                         -- mem_ck_n
        memory_mem_cke                        : out   std_logic;                                         -- mem_cke
        memory_mem_cs_n                       : out   std_logic;                                         -- mem_cs_n
        memory_mem_ras_n                      : out   std_logic;                                         -- mem_ras_n
        memory_mem_cas_n                      : out   std_logic;                                         -- mem_cas_n
        memory_mem_we_n                       : out   std_logic;                                         -- mem_we_n
        memory_mem_reset_n                    : out   std_logic;                                         -- mem_reset_n
        memory_mem_dq                         : inout std_logic_vector(31 downto 0)  := (others => 'X'); -- mem_dq
        memory_mem_dqs                        : inout std_logic_vector(3 downto 0)   := (others => 'X'); -- mem_dqs
        memory_mem_dqs_n                      : inout std_logic_vector(3 downto 0)   := (others => 'X'); -- mem_dqs_n
        memory_mem_odt                        : out   std_logic;                                         -- mem_odt
        memory_mem_dm                         : out   std_logic_vector(3 downto 0);                      -- mem_dm
        memory_oct_rzqin                      : in    std_logic                      := 'X';             -- oct_rzqin
        pio_led_external_connection_export    : out   std_logic_vector(7 downto 0);                      -- export
        reset_reset_n                         : in    std_logic                      := 'X';             -- reset_n
        pio_gpreg_external_connection_export  : out   std_logic_vector(31 downto 0);                    -- export
        pio_a_external_connection_export      : out   std_logic_vector(31 downto 0)  := (others => 'X'); -- in_port
        pio_b_external_connection_export      : out   std_logic_vector(31 downto 0);                    -- export
        pio_c_external_connection_export      : out   std_logic_vector(31 downto 0);                    -- export
        pio_d_external_connection_export      : out   std_logic_vector(31 downto 0);                    -- export
        pio_x_external_connection_export      : in    std_logic_vector(31 downto 0) := (others => 'X'); -- export
        pio_y_external_connection_export      : in    std_logic_vector(31 downto 0) := (others => 'X'); -- export
        pio_z_external_connection_export      : in    std_logic_vector(31 downto 0) := (others => 'X')  -- export
      );
	end component soc_system;

    component hps_reset 
        port (
	        probe       : in  std_logic_vector(0 downto 0);
	        source_clk  : in  std_logic;
	        source      : out std_logic_vector(2 downto 0));
    end component;

    component altera_edge_detector 
        generic(
            PULSE_EXT               : integer := 0; -- 0, 1 = edge detection generate single cycle pulse, >1 = pulse extended for specified clock cycle
            EDGE_TYPE               : integer := 0; -- 0 = falling edge, 1 or else = rising edge
            IGNORE_RST_WHILE_BUSY   : integer := 0  -- 0 = module internal reset will be default whenever rst_n asserted, 1 = rst_n request will be ignored while generating pulse out
        ); 
        port(
            clk         : in  std_logic;
            rst_n       : in  std_logic;
            signal_in   : in  std_logic;
            pulse_out   : out std_logic
        );
    end component;


    --------------------------------------------------------------------------------
    -- SIGNALS ---------------------------------------------------------------------
    --------------------------------------------------------------------------------
    signal rst_s                : std_logic := '0';


    signal f2h_cold_rst_s       : std_logic;
    signal f2h_debug_rst_s      : std_logic;
    signal f2h_warm_rst_s       : std_logic;
    signal h2f_rst_s            : std_logic;
    signal probe_s              : std_logic_vector(0 downto 0) := (others=>'1'); 

    signal hps_reset_req_s      : std_logic_vector(2 downto 0);

    signal led_s                : std_logic_vector(7 downto 0) := (others=>'0');

    -- Parallel I/O iface
    signal gpreg_s              : std_logic_vector(31 downto 0) := (others=>'0');
    signal a_s                  : std_logic_vector(31 downto 0) := (others=>'0');
    signal b_s                  : std_logic_vector(31 downto 0) := (others=>'0');
    signal c_s                  : std_logic_vector(31 downto 0) := (others=>'0');
    signal d_s                  : std_logic_vector(31 downto 0) := (others=>'0');
    signal x_s                  : std_logic_vector(31 downto 0) := (others=>'0');
    signal y_s                  : std_logic_vector(31 downto 0) := (others=>'0');
    signal z_s                  : std_logic_vector(31 downto 0) := (others=>'0');

    signal stm_events_s         : std_logic_vector(27 downto 0) := (others=>'0');
    
begin

    rst_s   <= not key_i(0);

    stm_events_s(1 downto 0)    <= key_i;
    stm_events_s(9 downto 2)    <= led_s;
    stm_events_s(13 downto 10)  <= sw_i;
    stm_events_s(27 downto 14)  <= (others=>'0');

    led_o   <= led_s;

    soc_system_u: component soc_system
    port map(
        clk_clk                                 => fpga_clk1_50, 
        hps_0_f2h_cold_reset_req_reset_n        => not f2h_cold_rst_s, 
        hps_0_f2h_debug_reset_req_reset_n       => not f2h_debug_rst_s,
        hps_0_f2h_stm_hw_events_stm_hwevents    => stm_events_s,
        hps_0_f2h_warm_reset_req_reset_n        => not f2h_warm_rst_s, 
        hps_0_h2f_reset_reset_n                 => h2f_rst_s,
        hps_0_hps_io_hps_io_emac1_inst_TX_CLK   => HPS_ENET_GTX_CLK,
        hps_0_hps_io_hps_io_emac1_inst_TXD0     => HPS_ENET_TX_DATA(0),
        hps_0_hps_io_hps_io_emac1_inst_TXD1     => HPS_ENET_TX_DATA(1), 
        hps_0_hps_io_hps_io_emac1_inst_TXD2     => HPS_ENET_TX_DATA(2), 
        hps_0_hps_io_hps_io_emac1_inst_TXD3     => HPS_ENET_TX_DATA(3), 
        hps_0_hps_io_hps_io_emac1_inst_RXD0     => HPS_ENET_RX_DATA(0), 
        hps_0_hps_io_hps_io_emac1_inst_MDIO     => HPS_ENET_MDIO,
        hps_0_hps_io_hps_io_emac1_inst_MDC      => HPS_ENET_MDC,
        hps_0_hps_io_hps_io_emac1_inst_RX_CTL   => HPS_ENET_RX_DV,
        hps_0_hps_io_hps_io_emac1_inst_TX_CTL   => HPS_ENET_TX_EN,
        hps_0_hps_io_hps_io_emac1_inst_RX_CLK   => HPS_ENET_RX_CLK,
        hps_0_hps_io_hps_io_emac1_inst_RXD1     => HPS_ENET_RX_DATA(1),
        hps_0_hps_io_hps_io_emac1_inst_RXD2     => HPS_ENET_RX_DATA(2),
        hps_0_hps_io_hps_io_emac1_inst_RXD3     => HPS_ENET_RX_DATA(3),
        hps_0_hps_io_hps_io_sdio_inst_CMD       => HPS_SD_CMD,
        hps_0_hps_io_hps_io_sdio_inst_D0        => HPS_SD_DATA(0),
        hps_0_hps_io_hps_io_sdio_inst_D1        => HPS_SD_DATA(1),
        hps_0_hps_io_hps_io_sdio_inst_CLK       => HPS_SD_CLK,
        hps_0_hps_io_hps_io_sdio_inst_D2        => HPS_SD_DATA(2),
        hps_0_hps_io_hps_io_sdio_inst_D3        => HPS_SD_DATA(3),
        hps_0_hps_io_hps_io_usb1_inst_D0        => HPS_USB_DATA(0), 
        hps_0_hps_io_hps_io_usb1_inst_D1        => HPS_USB_DATA(1),
        hps_0_hps_io_hps_io_usb1_inst_D2        => HPS_USB_DATA(2),
        hps_0_hps_io_hps_io_usb1_inst_D3        => HPS_USB_DATA(3),
        hps_0_hps_io_hps_io_usb1_inst_D4        => HPS_USB_DATA(4),
        hps_0_hps_io_hps_io_usb1_inst_D5        => HPS_USB_DATA(5),
        hps_0_hps_io_hps_io_usb1_inst_D6        => HPS_USB_DATA(6),
        hps_0_hps_io_hps_io_usb1_inst_D7        => HPS_USB_DATA(7),
        hps_0_hps_io_hps_io_usb1_inst_CLK       => HPS_USB_CLKOUT,
        hps_0_hps_io_hps_io_usb1_inst_STP       => HPS_USB_STP,
        hps_0_hps_io_hps_io_usb1_inst_DIR       => HPS_USB_DIR,
        hps_0_hps_io_hps_io_usb1_inst_NXT       => HPS_USB_NXT,
        hps_0_hps_io_hps_io_spim1_inst_CLK      => HPS_SPIM_CLK,
        hps_0_hps_io_hps_io_spim1_inst_MOSI     => HPS_SPIM_MOSI, 
        hps_0_hps_io_hps_io_spim1_inst_MISO     => HPS_SPIM_MISO,
        hps_0_hps_io_hps_io_spim1_inst_SS0      => HPS_SPIM_SS,
        hps_0_hps_io_hps_io_uart0_inst_RX       => HPS_UART_RX,
        hps_0_hps_io_hps_io_uart0_inst_TX       => HPS_UART_TX,
        hps_0_hps_io_hps_io_i2c0_inst_SDA       => HPS_I2C0_SDAT,
        hps_0_hps_io_hps_io_i2c0_inst_SCL       => HPS_I2C0_SCLK,
        hps_0_hps_io_hps_io_i2c1_inst_SDA       => HPS_I2C1_SDAT,
        hps_0_hps_io_hps_io_i2c1_inst_SCL       => HPS_I2C1_SCLK,
        hps_0_hps_io_hps_io_gpio_inst_GPIO09    => HPS_CONV_USB_N,
        hps_0_hps_io_hps_io_gpio_inst_GPIO35    => HPS_ENET_INT_N,
        hps_0_hps_io_hps_io_gpio_inst_GPIO40    => HPS_LTC_GPIO,
        hps_0_hps_io_hps_io_gpio_inst_GPIO53    => HPS_LED,
        hps_0_hps_io_hps_io_gpio_inst_GPIO54    => HPS_KEY,
        hps_0_hps_io_hps_io_gpio_inst_GPIO61    => HPS_GSENSOR_INT,
        memory_mem_a                            => HPS_DDR3_ADDR, 
        memory_mem_ba                           => HPS_DDR3_BA,
        memory_mem_ck                           => HPS_DDR3_CK_P,
        memory_mem_ck_n                         => HPS_DDR3_CK_N,
        memory_mem_cke                          => HPS_DDR3_CKE,
        memory_mem_cs_n                         => HPS_DDR3_CS_N,
        memory_mem_ras_n                        => HPS_DDR3_RAS_N,
        memory_mem_cas_n                        => HPS_DDR3_CAS_N,
        memory_mem_we_n                         => HPS_DDR3_WE_N,
        memory_mem_reset_n                      => HPS_DDR3_RESET_N, 
        memory_mem_dq                           => HPS_DDR3_DQ,
        memory_mem_dqs                          => HPS_DDR3_DQS_P,
        memory_mem_dqs_n                        => HPS_DDR3_DQS_N,
        memory_mem_odt                          => HPS_DDR3_ODT,
        memory_mem_dm                           => HPS_DDR3_DM,
        memory_oct_rzqin                        => HPS_DDR3_RZQ,
        pio_led_external_connection_export      => led_s, 
        reset_reset_n                           => '1',
        pio_gpreg_external_connection_export    => gpreg_s,
        pio_a_external_connection_export        => a_s,
        pio_b_external_connection_export        => b_s,
        pio_c_external_connection_export        => c_s,
        pio_d_external_connection_export        => d_s,
        pio_x_external_connection_export        => x_s,
        pio_y_external_connection_export        => y_s,
        pio_z_external_connection_export        => z_s
    );



    hps_reset_u: hps_reset
    port map(
	    probe       => probe_s,
	    source_clk  => fpga_clk1_50,
	    source      => hps_reset_req_s);

    pulse_cold_rst_u: altera_edge_detector 
    generic map(
        PULSE_EXT               => 6,
        EDGE_TYPE               => 1,
        IGNORE_RST_WHILE_BUSY   => 1
    ) 
    port map(
        clk         => fpga_clk1_50,
        rst_n       => h2f_rst_s,
        signal_in   => hps_reset_req_s(0),
        pulse_out   => f2h_cold_rst_s
    );

    pulse_warm_rst_u: altera_edge_detector 
    generic map(
        PULSE_EXT               => 2,
        EDGE_TYPE               => 1,
        IGNORE_RST_WHILE_BUSY   => 1
    ) 
    port map(
        clk         => fpga_clk1_50,
        rst_n       => h2f_rst_s,
        signal_in   => hps_reset_req_s(1),
        pulse_out   => f2h_warm_rst_s
    );

    pulse_debug_rst_u: altera_edge_detector 
    generic map(
        PULSE_EXT               => 32,
        EDGE_TYPE               => 1,
        IGNORE_RST_WHILE_BUSY   => 1
    ) 
    port map(
        clk         => fpga_clk1_50,
        rst_n       => h2f_rst_s,
        signal_in   => hps_reset_req_s(2),
        pulse_out   => f2h_debug_rst_s
    );

    --------------------------------------------------------------------------------
    -- APP
    --------------------------------------------------------------------------------
    group1_u: entity work.baskara_g1_core
    port map(
        en_i    => gpreg_s(0), 
        a_i     => a_s,
        b_i     => b_s,
        c_i     => c_s,
        op4_o   => x_s
    );


end behavioral;

