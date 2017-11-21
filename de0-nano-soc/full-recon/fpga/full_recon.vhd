----------------------------------------------------------------------
-- @file:   full_recon.vhd
-- @author: Lucas Manco
-- @date:   21 / 10 / 2017
-- @brief:  A simple logic to test the full reconfiguration on DE0-Nano
----------------------------------------------------------------------

-- IEEE Libraries
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity full_recon is
port(
    clk1_50 : in  std_logic;
    key_i   : in  std_logic_vector(1 downto 0);
    sw_i    : in  std_logic_vector(3 downto 0);
    led_o   : out std_logic_vector(7 downto 0));
end entity;

architecture behavioral of full_recon is

    signal rst_s        : std_logic := '0';
    signal count_s      : std_logic_vector(31 downto 0) := (others=>'0');
begin

    led_o(6 downto 1) <= (others=>'0');

    rst_s       <= not key_i(0);
    led_o(0)    <= not key_i(0);
    led_o (7)   <= count_s(24);

    count_p: process(clk1_50)
    begin
        if clk1_50'event and clk1_50 = '1' then
            if rst_s = '1' then
                count_s <= (others=>'0');
            else
                count_s <= std_logic_vector(unsigned(count_s) + 1);
            end if;
        end if;
    end process;


end behavioral;

