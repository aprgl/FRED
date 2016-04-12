library ieee;
use ieee.numeric_std.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

--============================================================================
-- PID Block
--============================================================================
-- PID!
-- ToDo: D
-- Version: 0.0.0 Initial Commit -Shaun
------------------------------------------------------------------------------

entity pid is
Port (
	-- Reset Signal
	rst_n_in		: in	std_logic;
	
	-- Clocks
	clk_in		: in	std_logic;
	
	-- Registers -- Fixed Point 4.4
	Kp_in	: in std_logic_vector(7 downto 0) := (others => '0');
	Ki_in	: in std_logic_vector(7 downto 0) := (others => '0');
	Kd_in	: in std_logic_vector(7 downto 0) := (others => '0');
	
	-- Sensor Signal
	signal_in	: in	std_logic_vector(15 downto 0);
	
	-- Ouput
	signal_out	: out	std_logic_vector(15 downto 0)
	 
	);
end entity pid;

architecture rtl of pid is

	signal temp    : std_logic_vector(27 downto 0)  :=  (others => '0');
	signal sum    : std_logic_vector(15 downto 0)  :=  (others => '0');
	signal s0, s1, s2, s3, s4, s5, s6, s7	: std_logic_vector(15 downto 0)  :=  (others => '0');

begin
    --========================================================================
    -- PID Calculation Logic
    --========================================================================
	 
    pid_proc: process (clk_in, rst_n_in, Kp_in, Ki_in, Kd_in, signal_in) begin
        if( rising_edge(clk_in) ) then
            
				if( rst_n_in = '0') then
					 temp <= (others => '0');
				end if;
				
				temp <= std_logic_vector(unsigned((signal_in & X"0") * Kp_in)+ unsigned((sum & X"0") * Ki_in));
				
				-- That's sum block you've got there
				sum <= std_logic_vector(unsigned(s0+s1+s2+s3+s4+s5+s6+s7));
				
				-- 8 stage integrator
				s0 <= signal_in;
				s1 <= s0;
				s2 <= s1;
				s3 <= s2;
				s4 <= s3;
				s5 <= s4;
				s6 <= s5;
				s7 <= s6;
				
			end if;
    end process pid_proc;
	 
	 signal_out <= temp(27 downto 12);
	 
end architecture rtl;