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
		pwm_uh_out <= std_logic_vector((unsigned("00"&X"00"&pwm_uh_in)*unsigned(torque_in&"00"&X"00"))/1024)(19 downto 10);
		pwm_ul_out <= std_logic_vector((unsigned("00"&X"00"&pwm_ul_in)*unsigned(torque_in&"00"&X"00"))/1024)(19 downto 10);
		pwm_vh_out <= std_logic_vector((unsigned("00"&X"00"&pwm_vh_in)*unsigned(torque_in&"00"&X"00"))/1024)(19 downto 10);
		pwm_vl_out <= std_logic_vector((unsigned("00"&X"00"&pwm_vl_in)*unsigned(torque_in&"00"&X"00"))/1024)(19 downto 10);
		pwm_wh_out <= std_logic_vector((unsigned("00"&X"00"&pwm_wh_in)*unsigned(torque_in&"00"&X"00"))/1024)(19 downto 10);
		pwm_wl_out <= std_logic_vector((unsigned("00"&X"00"&pwm_wl_in)*unsigned(torque_in&"00"&X"00"))/1024)(19 downto 10);
    end if;
    end process pwm_scale_proc;
    -------------------------------------------------------------------------
    
    end architecture rtl;