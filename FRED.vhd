library ieee;
use ieee.numeric_std.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

library vjtag;
library pll_core;

--============================================================================
--  Virtual JTAG Port Generation Block
--============================================================================
-- Generate a vuirt JTAG block to allow control and data to flow from
-- the FPGA under development.
-- Version: 0.0.0 Initial Commit - half dead time block - compiles -Shaun
------------------------------------------------------------------------------

entity FRED is
Port (

	clk_24	: in std_logic;
	RST_IN	: in std_logic;
	
	UH_OUT		: out	std_logic;
	UL_OUT		: out std_logic;
	VH_OUT		: out std_logic;
	VL_OUT		: out std_logic;
	WH_OUT		: out std_logic;
	WL_OUT		: out	std_logic;

	A_OUT			: out	std_logic_vector(1 downto 0);
	RES_OUT 		: out	std_logic_vector(1 downto 0);
	SAMPLE_OUT	: out std_logic;
	WR_OUT		: out std_logic;
	SCLK_OUT 	: out std_logic;
	MISO_IN		: in std_logic;
	MOSI_OUT		: out std_logic;
	
	brake_out	: out std_logic := '1';
	leds_out		: out std_logic_vector(7 downto 0) := X"00"
	
	--DEBUG_ENA	: in std_logic
	
	);
end entity FRED;

architecture rtl of FRED is

	-- Clocks
	signal clk_40, clk_600k, slow_clk : std_logic;
	-- Slow Clock Counter
	signal count : std_logic_vector(7 downto 0);

	-- Virtual JTAG Signals
	signal tdi_sig, tdo_sig, tck, sdr_valid, data_ready, load_dr	:	std_logic;
	signal ir	: std_logic_vector(7 downto 0);
	signal dr	: std_logic_vector(31 downto 0) := X"87654321";
	
	-- Sine lookup signals
	signal position, freerunning_position, digital_position 	: std_logic_vector(9 downto 0);

	-- Pwm signals
	signal	pwm_uh	: std_logic_vector(9 downto 0);
	signal	pwm_ul	: std_logic_vector(9 downto 0);
	signal	pwm_vh	: std_logic_vector(9 downto 0);
	signal	pwm_vl	: std_logic_vector(9 downto 0);
	signal	pwm_wh	: std_logic_vector(9 downto 0);
	signal	pwm_wl	: std_logic_vector(9 downto 0);

	signal	pwm_masked_ul, pwm_masked_uh	: std_logic_vector(9 downto 0);
	signal	pwm_masked_vl, pwm_masked_vh	: std_logic_vector(9 downto 0);
	signal	pwm_masked_wl, pwm_masked_wh	: std_logic_vector(9 downto 0);

	signal	torque_ul, torque_uh	: std_logic_vector(9 downto 0);
	signal	torque_vl, torque_vh	: std_logic_vector(9 downto 0);
	signal	torque_wl, torque_wh	: std_logic_vector(9 downto 0);
	
	signal	uh_signal, ul_signal	: std_logic := '0';
	signal	vh_signal, vl_signal	: std_logic := '0';
	signal	wh_signal, wl_signal	: std_logic := '0';
	
	-- GPIO signals
	signal gpio_out, gpio_in	: std_logic_vector(7 downto 0);	

	-- Resolver Signals
	signal resolver_signal	: std_logic_vector(23 downto 0);
	signal res					: std_logic_vector(1 downto 0) := (others => '0');
	signal address				: std_logic_vector(1 downto 0) := (others => '0');
	signal resolver_control	: std_logic_vector(3 downto 0);
	signal resolver_position : std_logic_vector(15 downto 0);
	signal sample_signal		: std_logic;

	-- Control Registers
	signal speed_ctrl_in 	: std_logic_vector(7 downto 0) := X"0F";		-- 0x01
	signal power_ctrl_in		: std_logic_vector(3 downto 0) := X"7";		-- 0x02
	signal nes_a				: std_logic	:= '0';									-- 0x03
	signal nes_b				: std_logic	:= '0';									-- 0x04
	signal charge				: std_logic	:= '0';									-- 0x05
	signal debug				: std_logic := '0';									-- 0x06
	signal debug_phase		: std_logic_vector(1 downto 0) := "00";		-- 0x07
	signal freerunning_ena	: std_logic := '1';									-- 0x0A
	signal phase_leds			: std_logic := '1';									-- 0x0B
	signal commutation_position : std_logic_vector(15 downto 0)	:= X"4074";-- 0x0C
	signal closed_loop_ena	: std_logic := '0';									-- 0x0D
	signal closed_loop_offset: std_logic_vector(9 downto 0) := "00"&X"00"; -- 0x0E
	signal raw_position		: std_logic_vector(9 downto 0); 					-- 0x0F
	signal speed_request		: std_logic_vector(7 downto 0);					-- 0x18
	
	-- Closed Loop Control Signals
	signal closed_loop_position		: std_logic_vector(9 downto 0) := "00"&X"00";
	
	-- Sensors
	signal motor_speed	: std_logic_vector(7 downto 0);
	signal torque			: std_logic_vector(9 downto 0);
	
	begin
 
-- Instantiate PLL to generate system clock
	pll : entity pll_core.pll_core(rtl)
	port map(
		refclk	=> clk_24,
		outclk_0	=>	clk_40,
		outclk_1	=>	clk_600k
	);
	 
-- The instantiation will create connect this block to the JTAG chain 
	virtual_jtag : entity vjtag.vjtag(rtl)
	port map(
		TDI 	=> tdi_sig,
		TDO 	=> tdo_sig,
		TCK	=> tck,
		IR_IN		=> ir,
		virtual_state_udr => data_ready,
		virtual_state_sdr => sdr_valid,
		virtual_state_cdr => load_dr
	 );

	tdo_sig <= dr(0);
	dr_proc: process (tck, tdi_sig, tdo_sig, load_dr, sdr_valid) begin
		if (rising_edge(tck)) then
			if (load_dr = '1') then
				if(ir = X"00") then
					dr <= X"12345678";
				elsif(ir = X"09") then
					dr <= X"0000" & resolver_signal(23 downto 8);
				elsif(ir = X"03") then
					dr <= X"0000000" & "000" & nes_a;
				elsif(ir = X"04") then
					dr <= X"0000000" & "000" & nes_b;
				elsif(ir = X"10") then
					dr <= X"000000" & motor_speed;
				elsif(ir = X"11") then
					dr <= X"00000" & "00"&torque;
				elsif(ir = X"12") then
					dr <= X"00000" & "00"&pwm_masked_uh;
				elsif(ir = X"13") then
					dr <= X"00000" & "00"&pwm_masked_ul;
				elsif(ir = X"14") then
					dr <= X"00000" & "00"&pwm_masked_vh;
				elsif(ir = X"15") then
					dr <= X"00000" & "00"&pwm_masked_vl;
				elsif(ir = X"16") then
					dr <= X"00000" & "00"&pwm_masked_wh;
				elsif(ir = X"17") then
					dr <= X"00000" & "00"&pwm_masked_wl;
				else
					dr <= X"00000000";
				end if;
			elsif (sdr_valid = '1') then
				dr <= (tdi_sig & dr(31 downto 1));
			end if;
		end if;
	end process;
	
	pwm_proc: process (tck, data_ready) begin
		if (data_ready = '1') then
			if(ir = X"01") then
				speed_ctrl_in <= dr(7 downto 0);
			elsif(ir = X"02") then
				power_ctrl_in <= dr(3 downto 0);
			elsif(ir = X"03") then
				nes_a <= dr(0);
			elsif(ir = X"04") then
				nes_b <= dr(0);
			elsif(ir = X"05") then
				charge <= dr(0);
			elsif(ir = X"06") then
				debug <= dr(0);
			elsif(ir = X"07") then
				debug_phase <= dr(1 downto 0);
			elsif(ir = X"08") then
				digital_position <= dr(9 downto 0);
			elsif(ir = X"0A") then
				freerunning_ena <= dr(0);
			elsif(ir = X"0B") then
				phase_leds <= dr(0);
			elsif(ir = X"0C") then			--12
				commutation_position <= dr(15 downto 0);
			elsif(ir = X"0D") then
				closed_loop_ena <= dr(0); --13
			elsif(ir = X"0E") then
				closed_loop_offset <= dr(9 downto 0);
			elsif(ir = X"0F") then
				raw_position <= dr(9 downto 0);
			elsif(ir = X"18") then
				speed_request <= dr(7 downto 0);
			end if;
		end if;
	end process;
	
	slow_clk_proc: process (clk_600k) begin
        if( rising_edge(clk_600k) ) then
            if( count = X"FE") then
                count <= (others => '0');
					 slow_clk <= not slow_clk;
            end if;
				count <= count + 1;
        end if;
    end process slow_clk_proc;

	 -- PWM Scaler
pwm_scale : entity work.pwm_scale(rtl)
	port map(
		rst_n_in		=> RST_IN,
		clk_in		=> not slow_clk,
		torque_in	=> torque,

		pwm_uh_in	=> pwm_uh,
		pwm_ul_in	=> pwm_ul,
		pwm_vh_in	=> pwm_vh,
		pwm_vl_in	=> pwm_vl,
		pwm_wh_in	=> pwm_wh,
		pwm_wl_in	=> pwm_wl,

		pwm_uh_out	=> torque_uh,
		pwm_ul_out	=> torque_ul,
		pwm_vh_out	=> torque_vh,
		pwm_vl_out	=> torque_vl,
		pwm_wh_out	=> torque_wh,
		pwm_wl_out	=> torque_wl
	);	
	
-- Torque Detector
torque_ctrl : entity work.torque_ctrl(rtl)
	port map(
		rst_n_in			=> RST_IN,
		clk_in			=> not slow_clk,
		speed_request	=> speed_request,
		speed_in			=> motor_speed,
		torque_out 		=> torque
	);	
 
-- Speed Detector
speed : entity work.speed(rtl)
	port map(
		rst_n_in			=> RST_IN,
		clk_in			=> not slow_clk,
		filter_len_in	=> X"00",
		resolver_in		=> resolver_signal(23 downto 8),
		speed_out		=> motor_speed
	);	
	
-- Free running open loop position indexer
counter : entity work.counter(rtl)
	port map(
		rst_n_in 	=> RST_IN,
		clk_in 		=> slow_clk,
		ena_in 		=> freerunning_ena,
		dir_in 		=> NES_A,
		prescale_in	=> SPEED_CTRL_IN,
		counter_out => freerunning_position
	 );

-- Closed loop controller
closed_loop : entity work.closed_loop(rtl)
	port map(
		rst_n_in 	=> RST_IN,
		clk_in 		=> slow_clk,
		ena_in 		=> closed_loop_ena,
		dir_in 		=> NES_A,
		commutation_position_in	=> commutation_position,
		position_in => resolver_signal(23 downto 8),
		position_out => closed_loop_position
	 );	 
	 
position <= closed_loop_position when closed_loop_ena = '1' else raw_position;

-- Instantiate and connect three phase generator
three_phase_lookup : entity work.three_phase(rtl)
	port map(
		-- FX2 interface
		rst_n_in    => RST_IN,
		clk_in      =>	clk_40,
		
		degree_in	=> position,
		
		sine_uh 		=> pwm_uh,
		sine_ul		=> pwm_ul,
		sine_vh 		=> pwm_vh,
		sine_vl		=> pwm_vl,
		sine_wh 		=> pwm_wh,
		sine_wl		=> pwm_wl
	);
	
	process (POWER_CTRL_IN, CHARGE, DEBUG_PHASE) begin
		if(CHARGE = '1') then
			pwm_masked_uh <= (others => '0');
			pwm_masked_ul <= "0011111111";
			pwm_masked_vh <= (others => '0');
			pwm_masked_vl <= "0011111111";
			pwm_masked_wh <= (others => '0');
			pwm_masked_wl <= "0011111111";
		elsif(DEBUG = '1') then
			if(DEBUG_PHASE = "001" and NES_A = '0') then		-- Debug Phase U Lock 3
				pwm_masked_uh <= "0111111000";
				pwm_masked_ul <= (others => '0');
				pwm_masked_vh <= (others => '0');
				pwm_masked_vl <= "0111111000";
				pwm_masked_wh <= (others => '0');
				pwm_masked_wl <= "0111111000";
			elsif(DEBUG_PHASE = "010" and NES_A = '0') then		-- Debug Phase V Lock 2
				pwm_masked_uh <= "0111111000";
				pwm_masked_ul <= (others => '0');
				pwm_masked_vh <= "0111111000";
				pwm_masked_vl <= (others => '0');
				pwm_masked_wh <= (others => '0');
				pwm_masked_wl <= "0111111000";
			elsif(DEBUG_PHASE = "011" and NES_A = '0') then		-- Debug Phase W Lock 1
				pwm_masked_uh <= (others => '0');
				pwm_masked_ul <= "0111111000";
				pwm_masked_vh <= "0111111000";
				pwm_masked_vl <= (others => '0');
				pwm_masked_wh <= (others => '0');
				pwm_masked_wl <= "0111111000";
			elsif(DEBUG_PHASE = "100" and NES_A = '0') then		-- Debug Phase W Lock 6
				pwm_masked_uh <= (others => '0');
				pwm_masked_ul <= "0111111000";
				pwm_masked_vh <= "0111111000";
				pwm_masked_vl <= (others => '0');
				pwm_masked_wh <= "0111111000";
				pwm_masked_wl <= (others => '0');
			elsif(DEBUG_PHASE = "101" and NES_A = '0') then		-- Debug Phase W Lock 5
				pwm_masked_uh <= (others => '0');
				pwm_masked_ul <= "0111111000";
				pwm_masked_vh <= (others => '0');
				pwm_masked_vl <= "0111111000";
				pwm_masked_wh <= "0111111000";
				pwm_masked_wl <= (others => '0');
			elsif(DEBUG_PHASE = "110" and NES_A = '0') then		-- Debug Phase W Lock 4
				pwm_masked_uh <= "0111111000";
				pwm_masked_ul <= (others => '0');
				pwm_masked_vh <= (others => '0');
				pwm_masked_vl <= "0111111000";
				pwm_masked_wh <= "0111111000";
				pwm_masked_wl <= (others => '0');
			else
				pwm_masked_uh <= (others => '0');
				pwm_masked_ul <= "1111111000";
				pwm_masked_vh <= (others => '0');
				pwm_masked_vl <= "1111111000";
				pwm_masked_wh <= (others => '0');
				pwm_masked_wl <= "1111111000";
			end if;
		else
			pwm_masked_uh <= torque_uh;
			pwm_masked_ul <= torque_ul;
			pwm_masked_vh <= torque_vh;
			pwm_masked_vl <= torque_vl;
			pwm_masked_wh <= torque_wh;
			pwm_masked_wl <= torque_wl;
		end if;
	end process;

uh_pwm : entity work.pwm(rtl)
	port map(
		rst_n_in		=> RST_IN,
		clk_in		=> clk_40,
		ena_in      => '1',
		value_in		=> pwm_masked_uh,
		pwm_out     => uh_signal,
		sample_out	=> sample_signal				--  Right now this is super sketchy because I can't be positive all the PWM blocks are synchronized but let's just pretend they are... 
	);

SAMPLE_OUT <= not sample_signal;

vh_pwm : entity work.pwm(rtl)
	port map(
		rst_n_in		=> RST_IN,
		clk_in		=> clk_40,
		ena_in      => '1',
		value_in		=> pwm_masked_vh,
		pwm_out     => vh_signal
	);

wh_pwm : entity work.pwm(rtl)
	port map(
		rst_n_in		=> RST_IN,
		clk_in		=> clk_40,
		ena_in      => '1',
		value_in		=> pwm_masked_wh,
		pwm_out     => wh_signal
	);

ul_pwm : entity work.pwm(rtl)
	port map(
		rst_n_in		=> RST_IN,
		clk_in		=> clk_40,
		ena_in      => '1',
		value_in		=> pwm_masked_ul,
		pwm_out     => ul_signal
	);

vl_pwm : entity work.pwm(rtl)
	port map(
		rst_n_in		=> RST_IN,
		clk_in		=> clk_40,
		ena_in      => '1',
		value_in		=> pwm_masked_vl,
		pwm_out     => vl_signal
	);

wl_pwm : entity work.pwm(rtl)
	port map(
		rst_n_in		=> RST_IN,
		clk_in		=> clk_40,
		ena_in      => '1',
		value_in		=> pwm_masked_wl,
		pwm_out     => wl_signal
	);	

	
	-- Dead Time Block 1.5uS minimum 25nS at 40MHz so 60 minimum, 100 for good measure gives us
u_dead_time: entity work.dead_time(rtl)
	port map(
	 rst_n_in => RST_IN,
    clk_in => clk_40,
    ena_in => '1',
	 pol_in => '1',						-- Invert polarity for optcoupler circuit
    high_side_in => uh_signal,
    low_side_in => ul_signal,
    dead_time_in => "01100100",
    high_side_out => UH_OUT,
    low_side_out => UL_OUT
	);
	
v_dead_time: entity work.dead_time(rtl)
	port map(
	 rst_n_in => RST_IN,
    clk_in => clk_40,
    ena_in => '1',
	 pol_in => '1',						-- Invert polarity for optcoupler circuit
    high_side_in => vh_signal,
    low_side_in => vl_signal,
    dead_time_in => "01100100",
    high_side_out => VH_OUT,
    low_side_out => VL_OUT
	);
	
w_dead_time: entity work.dead_time(rtl)
	port map(
	 rst_n_in => RST_IN,
    clk_in => clk_40,
    ena_in => '1',
	 pol_in => '1',						-- Invert polarity for optcoupler circuit
    high_side_in => wh_signal,
    low_side_in => wl_signal,
    dead_time_in => "01100100",
    high_side_out => WH_OUT,
    low_side_out => WL_OUT
	);
	
	resolver_control <= res & address;
	
resolver: entity work.spi(rtl)
	port map(
		reset_n => RST_IN,
		clk_in => clk_40,
		trigger_reading => slow_clk,
		spi_data_o => resolver_signal,
		
		-- Test rig
		switches_in => resolver_control,
		
		-- AD2S1210 
		A => A_OUT,
		RES => RES_OUT,
		-- sample_o => SAMPLE_OUT,
		WR_N_o => WR_OUT,
		SCLK => SCLK_OUT,
		MISO => MISO_IN,
		MOSI => MOSI_OUT		
	);

--==============================================
-- Stateless Signals
--==============================================
	leds_out <= (not uh_signal & not vh_signal & not wh_signal & not ul_signal & not vl_signal & not wl_signal & slow_clk & nes_b)
		when phase_leds = '1' else 
			(RST_IN & charge & debug & freerunning_ena & debug_phase & closed_loop_ena & dr(0));
	 brake_out <= not nes_b;
	 
end architecture rtl;