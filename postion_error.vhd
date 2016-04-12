library ieee;
use ieee.numeric_std.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

--============================================================================
-- Position Control Block
--============================================================================
-- Give a position error for the current position to requested position.
-- ToDo: Register output
-- Version: 0.0.0 Initial Commit -Shaun
------------------------------------------------------------------------------

entity position_error is
Port (
	-- Reset Signal
	rst_n_in		: in	std_logic;
	
	-- Clocks
	clk_in		: in	std_logic;
	
	-- Control Signals
	ena_in		: in	std_logic	:= '1';
	request_in	: in std_logic_vector(15 downto 0);
	
	-- Registers
	hysteresis_in	: in std_logic_vector(7 downto 0) := (others => '0');
	
	-- Sensor Signal
	position_in		: in	std_logic_vector(15 downto 0);
	
	-- Ouput
	error_out	: out	std_logic_vector(15 downto 0);
	dir_out		: out	std_logic
	 
	);
end entity position_error;

architecture rtl of position_error is
	signal temp    : std_logic_vector(15 downto 0)  :=  (others => '0');
begin
    --========================================================================
    -- Error Calculation Logic
    --========================================================================
	 
    error_proc: process (clk_in, rst_n_in, request_in, position_in, hysteresis_in) begin
        if( rising_edge(clk_in) ) then
            if( rst_n_in = '0') then
					 temp <= (others => '0');
					 dir_out <= '1';
            end if;

				if (ena_in = '1')	then
					if (position_in < (request_in + hysteresis_in)) then
						temp <= request_in - position_in;
						dir_out <= '1';
					elsif (position_in > (request_in + hysteresis_in)) then
						temp <= position_in - request_in;
						dir_out <= '0';
					end if;
				end if;
        end if;
    end process error_proc;
	 
	 error_out <= temp;
	 
end architecture rtl;