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
	ena_in		: in std_logic	:= '1';
	request_in	: in std_logic_vector(22 downto 0);
	home_set_in	: in std_logic := '0';
	
	-- Registers
	hysteresis_in	: in std_logic_vector(7 downto 0) := (others => '0');
	
	-- Sensor Signal
	resolver_in		: in	std_logic_vector(15 downto 0);
	
	-- Ouput
	error_out	: out	std_logic_vector(22 downto 0);
	dir_out		: out	std_logic;
	absolute_position_out	: out	std_logic_vector(22 downto 0)
	);
end entity position_error;

architecture rtl of position_error is
	
	signal temp    : std_logic_vector(22 downto 0)  :=  (others => '0');
	signal absolute_position	: std_logic_vector(22 downto 0)	:= (Others => '0');
	signal resolver_last	: std_logic_vector(15 downto 0)  :=  (others => '0');
	
begin
    --========================================================================
    -- Error Calculation Logic
    --========================================================================
	 
    error_proc: process (clk_in, rst_n_in, request_in, resolver_in, hysteresis_in, home_set_in) begin
        if( rising_edge(clk_in) ) then
            if( rst_n_in = '0') then
					 temp <= (others => '0');
					 dir_out <= '1';
            end if;
				
				if(home_set_in = '1') then
					absolute_position <= "011" & X"F" & resolver_in;
					
				elsif ((resolver_in > resolver_last) and ((resolver_in - resolver_last) < X"8FFF")) then
					absolute_position <= absolute_position + (resolver_in - resolver_last);
					
				elsif ((resolver_in > resolver_last) and ((resolver_in - resolver_last) > X"8FFF")) then
					absolute_position <= absolute_position - ((X"FFFF" - resolver_in) + resolver_last);
				
				elsif ((resolver_in < resolver_last) and ((resolver_last - resolver_in) < X"8FFF")) then
					absolute_position <= absolute_position - (resolver_last - resolver_in);
				
				elsif ((resolver_in < resolver_last) and ((resolver_last - resolver_in) > X"8FFF")) then
					absolute_position <= absolute_position + ((X"FFFF" - resolver_last) + resolver_in);
				end if;
				
				resolver_last <= resolver_in;
				
				if (ena_in = '1')	then
					if (absolute_position < request_in) then
						temp <= request_in - absolute_position;
						dir_out <= '1';
					elsif (absolute_position > request_in) then
						temp <= absolute_position - request_in;
						dir_out <= '0';
					end if;
				end if;
        end if;
    end process error_proc;
	 
	 error_out <= temp;
	 absolute_position_out <= absolute_position;
	 
end architecture rtl;