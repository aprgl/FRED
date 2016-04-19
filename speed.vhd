
library ieee;
use ieee.numeric_std.all;
use ieee.std_logic_1164.all;
--use ieee.std_logic_unsigned.all;

--============================================================================
--  Resolver to Speed Block
--============================================================================
-- Takes in a changing resolver signal and outputs a speed! So fancy.
-- Version: 0.0.0 Initial Commit -Shaun
------------------------------------------------------------------------------

entity speed is
Port (
    rst_n_in		: in 	std_logic	:= '1';
    clk_in			: in	std_logic;
	 filter_len_in	: in	std_logic_vector(7 downto 0)	:= (Others => '0');
    resolver_in	: in	std_logic_vector(15 downto 0) := (Others => '0');
    speed_out		: out	std_logic_vector(15 downto 0)	:= (Others => '0')
	 );
end entity speed;

architecture rtl of speed is
    
	 signal resolver_last : std_logic_vector(15 downto 0) := (Others => '0');
	 signal delta : std_logic_vector(16 downto 0) := (Others => '0');
	 
	 --signal speed_sum : std_logic_vector(23 downto 0) := (Others => '0');
	 --signal speed_avg : unsigned(23 downto 0) := (Others => '0');
    
    begin
	 
	 speed_calc_proc: process (clk_in, rst_n_in) begin
	-- If we have anew resolver signal
	 if rising_edge(clk_in) then
	 
		delta <= std_logic_vector(abs(signed("0"&resolver_last) - signed("0"&resolver_in)));
		
		-- speed_sum <= speed_sum + delta;
		resolver_last <= resolver_in;
		--speed_avg <= unsigned(speed_sum)/8;
		speed_out <= delta(15 downto 0);
    end if;
    end process speed_calc_proc;
    -------------------------------------------------------------------------
    
    --=======================================================================
    --  Stateless Signals
    --=======================================================================
    
	 --------------------------------------------------------------------------
    
    end architecture rtl;