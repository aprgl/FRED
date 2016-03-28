library ieee;
use ieee.numeric_std.all;
use ieee.std_logic_1164.all;
--use ieee.std_logic_unsigned.all;

--============================================================================
--  Torque Ctrl
--============================================================================
-- Takes in a torque value and ouptus a scaled PWM signal! So fancy.
-- Version: 0.0.0 Initial Commit -Shaun
------------------------------------------------------------------------------

entity pwm_scale is
Port (

	rst_n_in			: in 	std_logic	:= '1';
   clk_in			: in	std_logic;
	 
	torque_in		: in	std_logic_vector(9 downto 0);
	
	pwm_uh_in		: in	std_logic_vector(9 downto 0);
	pwm_ul_in		: in	std_logic_vector(9 downto 0);
	pwm_vh_in		: in	std_logic_vector(9 downto 0);
	pwm_vl_in		: in	std_logic_vector(9 downto 0);
	pwm_wh_in		: in	std_logic_vector(9 downto 0);
	pwm_wl_in		: in	std_logic_vector(9 downto 0);
	
	pwm_uh_out		: out	std_logic_vector(9 downto 0);
	pwm_ul_out		: out	std_logic_vector(9 downto 0);
	pwm_vh_out		: out	std_logic_vector(9 downto 0);
	pwm_vl_out		: out	std_logic_vector(9 downto 0);
	pwm_wh_out		: out	std_logic_vector(9 downto 0);
	pwm_wl_out		: out	std_logic_vector(9 downto 0)
	
	);
end entity pwm_scale;

architecture rtl of pwm_scale is

    begin
	
	 pwm_scale_proc: process (clk_in, rst_n_in, torque_in) begin
	 if rising_edge(clk_in) then
		pwm_uh_out <= std_logic_vector(unsigned(pwm_uh_in)/(1024-unsigned(torque_in)));
		pwm_ul_out <= std_logic_vector(unsigned(pwm_ul_in)/(1024-unsigned(torque_in)));
		pwm_vh_out <= std_logic_vector(unsigned(pwm_vh_in)/(1024-unsigned(torque_in)));
		pwm_vl_out <= std_logic_vector(unsigned(pwm_vl_in)/(1024-unsigned(torque_in)));
		pwm_wh_out <= std_logic_vector(unsigned(pwm_wh_in)/(1024-unsigned(torque_in)));
		pwm_wl_out <= std_logic_vector(unsigned(pwm_wl_in)/(1024-unsigned(torque_in)));
    end if;
    end process pwm_scale_proc;
    -------------------------------------------------------------------------
    
    end architecture rtl;