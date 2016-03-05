library ieee;
use ieee.numeric_std.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

--============================================================================
--  Dead Time Generation Block
--============================================================================
-- Generate dead time to avoid shoot through caused by high or low side being
-- commanded on before power device has fully turned off.
-- Version: 0.0.0 Initial Commit - haven't even tried to compile -Shaun
-- Version: 0.0.1 Fixed github linter errors - still haven't compiled -Shaun
-- Version: 0.0.2 Added enable signals -still haven't compiled -Inigo Montoya
------------------------------------------------------------------------------

entity dead_time is
Port (
    rst_n_in        : in    std_logic;
    clk_in          : in    std_logic;
    ena_in          : in    std_logic;
    high_side_in    : in    std_logic;
    low_side_in     : in    std_logic;
    dead_time_in    : in    std_logic_vector(7 downto 0);
    high_side_out   : out   std_logic;
    low_side_out    : out   std_logic
    );
end entity dead_time;

architecture rtl of dead_time is
    -- State Machine Signalss
    signal dead_time_counter    : std_logic_vector(7 downto 0)  :=  (others => '0');
    signal low_side_signal, high_side_signal, dead_time_complete    : std_logic := '0';
	 
    -- State Machine Signals
    type state_type is (state_reset,
        state_hold,
        state_standby,
        state_h, 
        state_l);

    signal state, next_state: state_type := state_reset; -- legal?
    
    begin

    --========================================================================
    --  State Machine Control
    --========================================================================
    -- State machine control block - reset and next state indexing
    --------------------------------------------------------------------------
    
    -- State machine control block - reset and next state indexing
    state_machine_ctrl: process (rst_n_in, clk_in) begin
    if (rst_n_in = '0') then
            state <= state_reset;       -- default state on reset
            elsif (rising_edge(clk_in)) then
            state <= next_state;        -- clocked change of state
            end if;
            end process state_machine_ctrl;

    -- State machine for our little dead time controller
    state_machine: process (state, 
        dead_time_complete,
        high_side_in,
        low_side_in) begin
    case state is
            -- If we're in a reset state, kill our outputs and assume they were
            -- previously commanded high, so reset our dead time counter to 0.
            when state_reset =>
            low_side_signal <= '0';
            high_side_signal <= '0';
            next_state <= state_hold;
            
            -- We're ready to start our dead time counter
            when state_hold =>
            low_side_signal <= '0';
            high_side_signal <= '0';
            if(dead_time_complete = '1') then
            next_state <= state_standby;
            else
            next_state <= state_hold;
            end if;

            -- Ready to drive an output
            when state_standby =>
            low_side_signal <= '0';
            high_side_signal <= '0';
            if (low_side_in = '1') then
            next_state <= state_l;
            elsif (high_side_in = '1') then
            next_state <= state_h;
            else
            next_state <= state_standby;
            end if; 

            -- Low side is being commanded on and we are not in dead time
            when state_l =>
            low_side_signal <= '1';
            high_side_signal <= '0';
            if (low_side_in = '1') then
            next_state <= state_l;
            else
            next_state <= state_reset;
            end if;

            -- High side is being commanded on and we are not in dead time
            when state_h =>
            low_side_signal <= '0';
            high_side_signal <= '1';
            if (high_side_in = '1') then
            next_state <= state_h;
            else
            next_state <= state_reset;
            end if;
            end case;
            end process state_machine;
    --------------------------------------------------------------------------
    
    --========================================================================
    --  Dead Time Counter Logic
    --========================================================================
    dead_time_counter_proc: process (state, clk_in) begin
    if (state = state_reset) then
		dead_time_complete <= '0';
		dead_time_counter <= (others => '0');
	 elsif (rising_edge(clk_in) and (state = state_hold)) then
		if(dead_time_counter < dead_time_in) then
			dead_time_counter <= dead_time_counter + 1;
		 else
			dead_time_complete <= '1';
		 end if;
    end if;

    end process dead_time_counter_proc;
    -------------------------------------------------------------------------
    
    --=======================================================================
    --  Stateless Signals
    --=======================================================================
    low_side_out <= '1' when (low_side_signal = '1' and ena_in = '1') else '0';
    high_side_out <= '1' when (high_side_signal = '1' and ena_in = '1') else '0';
    --------------------------------------------------------------------------
    
    end architecture rtl;