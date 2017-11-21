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


entity baskara_g1_core is
port(
    en_i    : in  std_logic;
    a_i     : in  std_logic_vector(31 downto 0);
    b_i     : in  std_logic_vector(31 downto 0);
    c_i     : in  std_logic_vector(31 downto 0);
    op4_o   : out std_logic_vector(31 downto 0));
end entity;

architecture behavioral of baskara_g1_core is

    signal a_s      : integer := 0; 
    signal b_s      : integer := 0; 
    signal c_s      : integer := 0; 
    signal op4_s    : integer := 0; 

begin

    a_s <= to_integer(signed(a_i));
    b_s <= to_integer(signed(b_i));
    c_s <= to_integer(signed(c_i));

    op4_s <= (b_s*b_s) - (4*a_s*c_s) when en_i = '1' else 0; 

    op4_o <= std_logic_vector(to_signed(op4_s, 32));

end behavioral;

