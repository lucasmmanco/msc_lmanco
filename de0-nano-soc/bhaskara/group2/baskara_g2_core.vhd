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


entity baskara_g2_core is
port(
    en_i    : in  std_logic;
    b_i     : in  std_logic_vector(31 downto 0);
    op4_i   : in  std_logic_vector(31 downto 0);
    op5_o   : out std_logic_vector(31 downto 0);
    op6_o   : out std_logic_vector(31 downto 0);
    op8_o   : out std_logic_vector(31 downto 0));
end entity;

architecture behavioral of baskara_g2_core is
    
    component sqrt
    port(
    	radical     : IN STD_LOGIC_VECTOR (31 DOWNTO 0);
    	q           : OUT STD_LOGIC_VECTOR (15 DOWNTO 0);
    	remainder   : OUT STD_LOGIC_VECTOR (16 DOWNTO 0));
    end component;

    signal b_s      : integer := 0; 
    signal op4_s    : std_logic_vector(31 downto 0) := (others=>'0');
    signal op5_s    : std_logic_vector(31 downto 0) := (others=>'0'); 
    signal op6_s    : integer := 0; 
    signal op8_s    : integer := 0; 

    signal sqrt_s   : std_logic_vector(15 downto 0) := (others=>'0');

begin

    op4_s   <= std_logic_vector(unsigned(signed(op4_i)));
    
    b_s     <= to_integer(signed(b_i));
    op6_s   <= (- b_s) when en_i = '1' else 0;
    op6_o   <= std_logic_vector(to_signed(op6_s, 32));
    
    op5_s(15 downto 0)  <= sqrt_s when en_i = '1' else (others=>'0');
    op5_o               <= op5_s;
    
    op8_s   <= to_integer(unsigned(op5_s)) + op6_s;
    op8_o   <= std_logic_vector(to_signed(op8_s, 32)) when op4_i(31) /= '1' else x"FFFF_FFFF";


    sqrt_u: sqrt
    port map(
    	radical     => op4_s,
    	q           => sqrt_s,
    	remainder   => open);

end behavioral;

