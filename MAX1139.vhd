library ieee;
use ieee.numeric_std.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

--============================================================================
--  Driver for MAX1139
--============================================================================
-- Driver for the MAX1139 ADC for reading motor phase currents.
-- Version: 0.0.2 - Rewriting large portions to be more generic
-- Version: 0.0.1 - Actually cleaning up the documentation. Trying to get this
--     code to a working level. There's a bit on nonsense in here. -Shaun
------------------------------------------------------------------------------

entity MAX1139 is
Port (
	-- clock and reset signals
	clk_in			: in	std_logic;
	rst_n_in		: in	std_logic;

	-- I2C data and flow control
	data_rdy_out	: out	std_logic;
	tx_data_out		: out	std_logic_vector(7 downto 0);
	rx_data_in		: in	std_logic_vector(7 downto 0);
	i2c_busy_in		: in	std_logic;
	rw_out			: out 	std_logic;

	-- ADC Readings
	ain0_out		: out 	std_logic_vector(9 downto 0);
	ain1_out		: out 	std_logic_vector(9 downto 0);
	ain2_out		: out 	std_logic_vector(9 downto 0);
	ain3_out		: out 	std_logic_vector(9 downto 0);
	ain4_out		: out 	std_logic_vector(9 downto 0);
	ain5_out		: out 	std_logic_vector(9 downto 0);
	ain6_out		: out 	std_logic_vector(9 downto 0);
	ain7_out		: out 	std_logic_vector(9 downto 0);
	ain8_out		: out 	std_logic_vector(9 downto 0);
	ain9_out		: out 	std_logic_vector(9 downto 0);
	ain10_out		: out 	std_logic_vector(9 downto 0);
	ain11_out		: out 	std_logic_vector(9 downto 0);
	
	-- trigger reading
	trigger_in		: in	std_logic

	);
end entity MAX1139;

architecture rtl of MAX1139 is
	
	signal adc_reg		: std_logic_vector(4 downto 0) := (Others => '0');

	signal ain0			: std_logic_vector(9 downto 0) := (Others => '0');
	signal ain1			: std_logic_vector(9 downto 0) := (Others => '0');
	signal ain2			: std_logic_vector(9 downto 0) := (Others => '0');
	signal ain3			: std_logic_vector(9 downto 0) := (Others => '0');
	signal ain4			: std_logic_vector(9 downto 0) := (Others => '0');
	signal ain5			: std_logic_vector(9 downto 0) := (Others => '0');
	signal ain6			: std_logic_vector(9 downto 0) := (Others => '0');
	signal ain7			: std_logic_vector(9 downto 0) := (Others => '0');
	signal ain8			: std_logic_vector(9 downto 0) := (Others => '0');
	signal ain9			: std_logic_vector(9 downto 0) := (Others => '0');
	signal ain10		: std_logic_vector(9 downto 0) := (Others => '0');
	signal ain11		: std_logic_vector(9 downto 0) := (Others => '0');

	signal u_current	: std_logic_vector(9 downto 0) := (Others => '0');
	signal v_current	: std_logic_vector(9 downto 0) := (Others => '0');
	signal w_current	: std_logic_vector(9 downto 0) := (Others => '0');
	 
	-- State Machine Signals
	type state_type is (	state_reset,	-- External/POR Reset State
							state_setup,	-- 
							s_hold1,
							state_config,
							s_hold2,
							state_read,
							s_hold3,
							state_done);
	signal state, next_state: state_type := state_reset; -- legal?

	signal i2c_busy_last	: std_logic;
	
	begin

	--========================================================================
	--  State Machine Control
	--========================================================================
	-- State machine control block - reset and next state indexing
	--------------------------------------------------------------------------
	
	-- State machine control block - reset and next state indexing
	state_machine_ctrl: process (rst_n_in, clk_in) begin
	if (rising_edge(clk_in)) then
		if (rst_n_in = '0') then
			state <= state_reset;   -- default state on reset
		else
			state <= next_state;	-- clocked change of state
		end if;
	end if;
	end process state_machine_ctrl;

	-- State machine reading control block - need to read multiple registers
	state_machine_read_ctrl: process (rst_n_in, clk_in, i2c_busy_in, i2c_busy_last, state) begin
	if (rising_edge(clk_in)) then
		if (state = state_reset) then
			-- On reset clear all variables
			adc_reg <= (Others => '0');
			ain0 <= (Others => '0');
			ain1 <= (Others => '0');
			ain2 <= (Others => '0');
			ain3 <= (Others => '0');
			ain4 <= (Others => '0');
			ain5 <= (Others => '0');
			ain6 <= (Others => '0');
			ain7 <= (Others => '0');
			ain8 <= (Others => '0');
			ain9 <= (Others => '0');
			ain10 <= (Others => '0');
			ain11 <= (Others => '0');

		elsif (state = s_hold3) then
			i2c_busy_last <= i2c_busy_in; -- pulled from eewiki's spi to i2c code
			if (i2c_busy_last = '1' and i2c_busy_in = '0') then	-- Hot&Fresh Data
				adc_reg <= adc_reg + '1';
				if (adc_reg = "0000") then
					ain0(9 downto 8) <= rx_data_in(1 downto 0);
				elsif (adc_reg = "00001") then
					ain0(7 downto 0) <= rx_data_in;
				elsif (adc_reg = "00010") then
					ain1(9 downto 8) <= rx_data_in(1 downto 0);
				elsif (adc_reg = "00011") then
					ain1(7 downto 0) <= rx_data_in;
				elsif (adc_reg = "00100") then
					ain2(9 downto 8) <= rx_data_in(1 downto 0);
				elsif (adc_reg = "00101") then
					ain2(7 downto 0) <= rx_data_in;
				elsif (adc_reg = "00110") then
					ain3(9 downto 8) <= rx_data_in(1 downto 0);
				elsif (adc_reg = "00111") then
					ain3(7 downto 0) <= rx_data_in;
				elsif (adc_reg = "01000") then
					ain4(9 downto 8) <= rx_data_in(1 downto 0);
				elsif (adc_reg = "01001") then
					ain4(7 downto 0) <= rx_data_in;
				elsif (adc_reg = "01010") then
					ain5(9 downto 8) <= rx_data_in(1 downto 0);
				elsif (adc_reg = "01011") then
					ain5(7 downto 0) <= rx_data_in;
				elsif (adc_reg = "01100") then
					ain6(9 downto 8) <= rx_data_in(1 downto 0);
				elsif (adc_reg = "01101") then
					ain6(7 downto 0) <= rx_data_in;
				elsif (adc_reg = "01110") then
					ain7(9 downto 8) <= rx_data_in(1 downto 0);
				elsif (adc_reg = "01111") then
					ain7(7 downto 0) <= rx_data_in;
				elsif (adc_reg = "10000") then
					ain8(9 downto 8) <= rx_data_in(1 downto 0);
				elsif (adc_reg = "10001") then
					ain8(7 downto 0) <= rx_data_in;
				elsif (adc_reg = "10010") then
					ain9(9 downto 8) <= rx_data_in(1 downto 0);
				elsif (adc_reg = "10011") then
					ain9(7 downto 0) <= rx_data_in;
				elsif (adc_reg = "10010") then
					ain10(9 downto 8) <= rx_data_in(1 downto 0);
				elsif (adc_reg = "10011") then
					ain10(7 downto 0) <= rx_data_in;
				elsif (adc_reg = "10100") then
					ain11(9 downto 8) <= rx_data_in(1 downto 0);
				elsif (adc_reg = "10101") then
					ain11(7 downto 0) <= rx_data_in;
				end if;
			end if;
		elsif (state = state_done) then
			ain0_out <= ain0;
			ain1_out <= ain1;
			ain2_out <= ain2;
			ain3_out <= ain3;
			ain4_out <= ain4;
			ain5_out <= ain5;
			ain6_out <= ain6;
			ain7_out <= ain7;
			ain8_out <= ain8;
			ain9_out <= ain9;
			ain10_out <= ain10;
			ain11_out <= ain11;
		end if;
	end if;
	end process state_machine_read_ctrl;
	
	-- State machine
	state_machine: process (state, i2c_busy_in, adc_reg, trigger_in) begin
	case state is
		-- In a reset state -- reassert configuration registers.
		when state_reset =>
			if ( i2c_busy_in = '0' and trigger_in = '1') then		-- If the I2C lines are free
				next_state <= state_setup;
			else
				next_state <= state_reset;
			end if;

			-- OUTPUTS --			
			tx_data_out <= (Others => '0');
			data_rdy_out <= '0';
			rw_out <= '0';					-- Writing
			
		when state_setup =>
			if (i2c_busy_in = '0') then
				next_state <= state_setup;
			else
				next_state <= s_hold1;
			end if;

			-- OUTPUTS --
			-- REG SEL2 SEL1 SEL0 CLK BIP/UNI RST X
			-- REG = 1 - Setup Register
			-- SEL[2:0] = (5 - Internal Reference Voltage (2.046))
			-- CLK = 0 - Internal Clock
			-- BIP/UNI = 0 - Unipolar Inputs
			-- RST = 1 - Don't Reboot!
			-- X = 0 - Don't Care Bit
			tx_data_out <=  "11010010";
			data_rdy_out <= '1';
			rw_out <= '0';					-- Writing
					
		when s_hold1 =>
			if (i2c_busy_in = '1') then
				next_state <= s_hold1;
			else
				next_state <= state_config;
			end if;

			-- OUTPUTS --
			tx_data_out <=  "10000010";
			data_rdy_out <= '0';
			rw_out <= '0';					-- Writing
		
		when state_config =>
			if (i2c_busy_in = '0') then
				next_state <= state_config;
			else
				next_state <= s_hold2;
			end if;
			
			-- OUTPUTS --
			-- REG SCAN1 SCAN0 CS3 CS2 CS1 CS0 SGL/DIF
			-- REG = 0 - Configuration Register
			-- SCAN[1:0] = 0 - Scan from A0 to A2
			-- CS[3:0] = 2 - Set the upper bound of the scan at Ch2
			-- SGL/DIF = 1 - Single Ended			
			tx_data_out <=  "00010111";
			data_rdy_out <= '1';
			rw_out <= '0';					-- Writing
			
		when s_hold2 =>
			if (i2c_busy_in = '1') then
				next_state <= s_hold2;
			else
				next_state <= state_read;
			end if;

			-- OUTPUTS --
			tx_data_out <= (Others => '0');
			data_rdy_out <= '0';
			rw_out <= '0';					-- Writing
		
		when state_read =>
			if (i2c_busy_in = '0') then
				next_state <= state_read;
			else
				next_state <= s_hold3;
			end if;
			
			-- OUTPUTS --
			tx_data_out <= (Others => '0');
			data_rdy_out <= '1';
			rw_out <= '1';					-- Reading
			
		when s_hold3 =>
			if (i2c_busy_in = '1') then
				next_state <= s_hold3;
			elsif (adc_reg < "10101") then
				next_state <= state_read;
			else
				next_state <= state_done;
			end if;

			-- OUTPUTS --
			tx_data_out <= (Others => '0');
			data_rdy_out <= '1';
			rw_out <= '1';					-- Reading
			
		when state_done =>
			if(trigger_in = '0') then		-- If the I2C lines are free
				next_state <= state_reset;
			else
				next_state <= state_done;
			end if;

			-- OUTPUTS --			
			tx_data_out <= (Others => '0');
			data_rdy_out <= '0';
			rw_out <= '0';					-- Writing

		end case;
	end process state_machine;
	-------------------------------------------------------------------------
	
	--=======================================================================
	--  Stateless Signals
	--=======================================================================

	-------------------------------------------------------------------------
	
	end architecture rtl;
