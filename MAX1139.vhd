library ieee;
use ieee.numeric_std.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

--============================================================================
--  Phase Currents from MAX1139
--============================================================================
-- Generate dead time to avoid shoot through caused by high or low side being
-- commanded on before power device has fully turned off.
-- Version: 0.0.0 Initial Commit - haven't even tried to compile -Shaun
-- Version: 0.0.1 Fixed github linter errors - still haven't compiled -Shaun
-- Version: 0.0.2 Added enable signals -still haven't compiled -Inigo Montoya
------------------------------------------------------------------------------

entity MAX1139 is
Port (
	clk_in			: in	std_logic;
	rst_n_in			: in	std_logic;
	data_rdy_out	: out	std_logic;
	tx_data_out		: out	std_logic_vector(7 downto 0);
	rx_data_in		: in	std_logic_vector(7 downto 0);
	i2c_busy_in		: in	std_logic;
	rw_out			: out std_logic;

	u_current_out	: out	std_logic_vector(9 downto 0);
	v_current_out	: out	std_logic_vector(9 downto 0);
	w_current_out	: out	std_logic_vector(9 downto 0)
	);
end entity MAX1139;

architecture rtl of MAX1139 is

	signal adc_reg		: std_logic_vector(3 downto 0) := (Others => '0');
   signal u_current	: std_logic_vector(9 downto 0) := (Others => '0');
	 
    -- State Machine Signals
    type state_type is (state_reset,
									state_setup,
									s_hold1,
									state_config,
									s_hold2,
									state_read,
									state_latch);
    signal state, next_state: state_type := state_reset; -- legal?
    
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
	   end if;
		state <= next_state;				-- clocked change of state
	end if;
	end process state_machine_ctrl;

	-- State machine
	state_machine: process (state, i2c_busy_in) begin
	case state is
		-- If we're in a reset state, let's reassert our configuration registers.
		when state_reset =>
			if( i2c_busy_in = '0') then		-- If the I2C lines are free
				next_state <= state_setup;
			else
				next_state <= state_reset;
			end if;
			tx_data_out <= (Others => '0');
			data_rdy_out <= '0';
			rw_out <= '0';						-- Writing
			adc_reg	<= (Others => '0');
         
		when state_setup =>
		-- REG SEL2 SEL1 SEL0 CLK BIP/UNI RST X
		-- REG = 1 - Setup Register
		-- SEL[2:0] = 0 - VDD as Reference Voltage (3.3)
		-- CLK = 0 - Internal Clock
		-- BIP/UNI = 0 - Unipolar Inputs
		-- RST = 1 - Don't Reboot!
		-- X = 0 - Don't Care Bit
			tx_data_out <=  "10000010";
			data_rdy_out <= '1';
			rw_out <= '0';						-- Writing
			next_state <= s_hold1;
			
		
		when s_hold1 =>
			tx_data_out <=  "10000010";
			data_rdy_out <= '0';
			rw_out <= '0';						-- Writing
			if (i2c_busy_in = '1') then
				next_state <= s_hold1;
			else
				next_state <= state_config;
			end if;
		
		when state_config =>
		-- REG SCAN1 SCAN0 CS3 CS2 CS1 CS0 SGL/DIF
		-- REG = 0 - Configuration Register
		-- SCAN[1:0] = 0 - Scan from A0 to A2
		-- CS[3:0] = 2 - Set the upper bound of the scan at Ch2
		-- SGL/DIF = 1 - Single Ended
			tx_data_out <=  "00000101";
			data_rdy_out <= '1';
			next_state <= s_hold2;
		
		when s_hold2 =>
			tx_data_out <=  "00000101";
			data_rdy_out <= '0';
			rw_out <= '0';						-- Writing
			if (i2c_busy_in = '1') then
				next_state <= s_hold2;
			else
				next_state <= state_read;
			end if;

		when state_read =>
			tx_data_out <= (Others => '0');
			data_rdy_out <= '1';
			rw_out <= '1';					-- Reading
			next_state <= state_latch;
			adc_reg <= adc_reg + '1';
		
		when state_latch =>
			tx_data_out <= (Others => '0');
			data_rdy_out <= '1';
			rw_out <= '1';					-- Reading
			u_current <= rx_data_in & "00";
			if (i2c_busy_in = '1') then
				next_state <= state_latch;
			elsif (adc_reg < "0011") then
				next_state <= state_read;
			else
				next_state <= state_reset;
			end if;

		end case;

      end process state_machine;
    -------------------------------------------------------------------------
    
    --=======================================================================
    --  Stateless Signals
    --=======================================================================
    u_current_out <= u_current;
    --------------------------------------------------------------------------
    
    end architecture rtl;