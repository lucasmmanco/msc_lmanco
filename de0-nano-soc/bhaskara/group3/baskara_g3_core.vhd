----------------------------------------------------------------------
-- @file:   baskara_g1_core.vhd
-- @author: Lucas Manco
-- @date:   04 / 11 / 2017
-- @brief:  Operations of the first group of baskara
----------------------------------------------------------------------

-- IEEE Libraries
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity baskara_g3_core is
port(
    en_i    : in  std_logic;
    a_i     : in  std_logic_vector(31 downto 0);
    op5_i   : in  std_logic_vector(31 downto 0);
    op6_i   : in  std_logic_vector(31 downto 0);
    op8_i   : in  std_logic_vector(31 downto 0);
    x1_o    : out std_logic_vector(31 downto 0);
    x2_o    : out std_logic_vector(31 downto 0));
end entity;

architecture behavioral of baskara_g3_core is

    signal a_s      : integer := 0; 
    signal op8_s    : integer := 0; 
    signal op5_s    : integer := 0; 
    signal op6_s    : integer := 0;
    signal x1_s     : integer := 0;
    signal x2_s     : integer := 0;

begin

    a_s     <= to_integer(signed(a_i));
    op8_s   <= to_integer(signed(op8_i));
    op5_s   <= to_integer(signed(op5_i));
    op6_s   <= to_integer(signed(op6_i));

    x1_s    <= op8_s/(2*a_s) when en_i = '1' else 0;
    x2_s    <= (op6_s - op5_s)/(2*a_s) when en_i = '1' else 0;

    x1_o    <= std_logic_vector(to_signed(x1_s, 32));
    x2_o    <= std_logic_vector(to_signed(x2_s, 32));

end behavioral;

