library ieee;
use ieee.numeric_std.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

--============================================================================
--  Three Phase Shenanigans
--============================================================================
-- Generate three phase 10 bit high and low signals from a 21845 bit input.
-- Version: 0.0.0 Initial Commit -Shaun
------------------------------------------------------------------------------

entity three_phase is
	Port (
		rst_n_in    :	in		std_logic;
		clk_in      :	in		std_logic;

		degree_in	:	in 	std_logic_vector(9 downto 0) := (others => '0');

		sine_uh 		:	out std_logic_vector(9 downto 0) := (others => '0');
		sine_ul		:  out std_logic_vector(9 downto 0) := (others => '0');
		sine_vh 		:	out std_logic_vector(9 downto 0) := (others => '0');
		sine_vl		:  out std_logic_vector(9 downto 0) := (others => '0');
		sine_wh 		:	out std_logic_vector(9 downto 0) := (others => '0');
		sine_wl		:  out std_logic_vector(9 downto 0) := (others => '0')
		
	);
end entity three_phase;

architecture rtl of three_phase is

	signal degree_registered	: std_logic_vector(10 downto 0)	:= (others => '0');
	signal sine_lookup 			: std_logic_vector(10 downto 0)	:= (others => '0');
	signal sine_u_internal, sine_v_internal, sine_w_internal 	: std_logic_vector(10 downto 0) := "011"&X"FF";
	signal sine, sine_u_reg, sine_v_reg, sine_w_reg					: std_logic_vector(10 downto 0) := "011"&X"FF";

	TYPE STATE_TYPE IS (state_reset, state_u, state_v, state_w);
	SIGNAL state   : STATE_TYPE;
	
begin
	--=============================================================================================
	--  SINE GENERATION
	--=============================================================================================
	-- Thee phase sine lookup
	-----------------------------------------------------------------------------------------------
	sine_lookup_proc: process (clk_in, sine_lookup, rst_n_in) begin
		if rising_edge(clk_in) then
			CASE state IS
				WHEN state_reset =>
					sine_lookup <= degree_registered;
				WHEN state_u =>
					sine_u_internal <= sine;
					sine_lookup <= degree_registered + 341;
				WHEN state_v =>
					sine_v_internal <= sine;
					sine_lookup <= degree_registered + 683;
				WHEN state_w =>
					sine_w_internal <= sine;
			END CASE;
		elsif falling_edge(clk_in) then
			if(rst_n_in = '0') then
				state <= state_reset;
			else
				CASE state IS
					WHEN state_reset =>
						state <= state_u;
						degree_registered <= "0"&degree_in;
					WHEN state_u =>
						state <= state_v;
					WHEN state_v =>
						state <= state_w;
					WHEN state_w =>
						sine_u_reg <= sine_u_internal;
						sine_v_reg <= sine_v_internal;
						sine_w_reg <= sine_w_internal;
						state <= state_reset;
					END CASE;
			end if;
		end if;
	end process sine_lookup_proc;
 
	--=============================================================================================
	--  SINE ROM
	--=============================================================================================
	-- sine lookup table 
	-- Currently 21845 steps/resolution at 11 bit magnitude resolution
	-----------------------------------------------------------------------------------------------
	lookup : entity work.sine_lookup(SYN)
	port map(
		address 	=> sine_lookup(9 downto 0),
		clock    => not clk_in,
		q        => sine
	);

	-- High Side Signals
	sine_uh <= sine_u_reg(9 downto 0) when sine_u_reg(10) = '1' else (others => '0');
	sine_vh <= sine_v_reg(9 downto 0) when sine_v_reg(10) = '1' else (others => '0');
	sine_wh <= sine_w_reg(9 downto 0) when sine_w_reg(10) = '1' else (others => '0');
	
	-- Low Side Signals
	sine_ul <= not sine_u_reg(9 downto 0) when sine_u_reg(10) = '0' else (others => '0');
	sine_vl <= not sine_v_reg(9 downto 0) when sine_v_reg(10) = '0' else (others => '0');
	sine_wl <= not sine_w_reg(9 downto 0) when sine_w_reg(10) = '0' else (others => '0');
	
end architecture rtl;