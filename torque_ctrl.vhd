library ieee;
use ieee.numeric_std.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

--============================================================================
--  Resolver to Speed Block
--============================================================================
-- Takes in a changing resolver signal and outputs a speed! So fancy.
-- Version: 0.0.0 Initial Commit -Shaun
------------------------------------------------------------------------------

entity torque_ctrl is
Port (
    rst_n_in		: in 	std_logic	:= '1';
    clk_in			: in	std_logic;
	 speed_request : in 	std_logic_vector(15 downto 0)	:= (Others => '0');
    speed_in		: in	std_logic_vector(15 downto 0)	:= (Others => '0');
    torque_out		: out	std_logic_vector(9 downto 0)	:= (Others => '0')
	 );
end entity torque_ctrl;

architecture rtl of torque_ctrl is
    
	 signal torque_state	: std_logic_vector(9 downto 0)	:= (Others => '0');
	 
    begin
	 
	 speed_calc_proc: process (clk_in, rst_n_in) begin
	-- If we have anew resolver signal
	 if rising_edge(clk_in) then
		--delta <= std_logic_vector(abs(signed("0"&resolver_last) - signed("0"&resolver_in)));
		--resolver_last <= resolver_in;
		--speed_out <= X"0000" & delta(7 downto 0);
		if(speed_request > speed_in and torque_state < "11"&X"ff") then
			torque_state <= torque_state + 1;
		elsif(speed_request < speed_in and torque_state > 0) then
			torque_state <= torque_state - 1;
		end if;
    end if;

    end process speed_calc_proc;
    -------------------------------------------------------------------------
    
    --=======================================================================
    --  Stateless Signals
    --=======================================================================
		torque_out <= torque_state;
	 --------------------------------------------------------------------------
    
    end architecture rtl;