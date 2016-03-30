library ieee;
use ieee.numeric_std.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

--============================================================================
-- Closed Loop Controller Block
--============================================================================
-- Command a 10-Bit Position from a 16-Bit Resolver Position 
-- Version: 0.0.0 Initial Commit -Shaun
------------------------------------------------------------------------------

entity closed_loop is
Port (
	-- Reset Signal
	rst_n_in		: in	std_logic;
	
	-- Clocks
	clk_in		: in	std_logic;
	
	-- Control Signals
	ena_in		: in	std_logic;
	dir_in		: in	std_logic;
	
	-- Registers
	commutation_position_in	: in std_logic_vector(15 downto 0);
	
	-- Sensor Signal
	position_in		: in	std_logic_vector(15 downto 0);
	
	-- Command Ouput
	position_out	: out	std_logic_vector(9 downto 0)
	 
	);
end entity closed_loop;

architecture rtl of closed_loop is
	signal temp    : std_logic_vector(31 downto 0)  :=  (others => '0');
begin
    --========================================================================
    -- Counter Logic
    --========================================================================
	 
    ctrl_proc: process (clk_in, rst_n_in,commutation_position_in, position_in) begin
        if( rising_edge(clk_in) ) then
            if( rst_n_in = '0') then
					 temp <= (others => '0');
            end if;

				if (ena_in = '1')	then
					if (dir_in = '1') then
						temp <= std_logic_vector(67043328-(unsigned(position_in+commutation_position_in+X"1555")*((67043328)/(65536/3)))); --67043328 = 1024<<16, +90 degrees
					else
						temp <= std_logic_vector(67043328-(unsigned(position_in+commutation_position_in-X"1555")*((67043328)/(65536/3)))); --67043328 = 1024<<16, -90 degrees
					end if;
				end if;
        end if;
    end process ctrl_proc;
	 
	 position_out <= temp(25 downto 16);
	 
end architecture rtl;