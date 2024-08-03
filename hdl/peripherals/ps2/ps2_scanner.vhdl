-------------------------------------------------------------
-- ps2_scanner
-- Synchronizes ps2 signals and reads input data
-------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_misc.all;

entity ps2_scanner is
    port (
            clk         : in std_logic;
            rst         : in std_logic;
            ps2_clk     : in std_logic;
            ps2_data    : in std_logic;
            rx_done     : out std_logic;
            rx_data_o   : out std_logic_vector(7 downto 0)
         );
end ps2_scanner;


architecture rtl of ps2_scanner is
    type statetype is (IDLE, RECEIVE, DONE);
    signal state : statetype;
    signal bit_cnt : unsigned(3 downto 0);
    signal rxdata : std_logic_vector(10 downto 0);
    signal ps2_clk_negedge : std_logic;
    signal edge_ff : std_logic;
    signal parity_bit : std_logic;

    -- Sychronized Signals
    signal ps2_clk_sync : std_logic;
    signal ps2_data_sync : std_logic;

    component synchronizer
        port(clk      : in std_logic;
             rst      : in std_logic;
             sig_i    : in std_logic;
             sig_sync : out std_logic);
    end component;
begin

    -- Synchronize PS2 signals
    SYNC_PS2_CLK: synchronizer
        port map ( clk => clk,
                   rst => rst,
                   sig_i => ps2_clk,
                   sig_sync => ps2_clk_sync );
    SYNC_PS2_DATA: synchronizer
        port map ( clk => clk,
                   rst => rst,
                   sig_i => ps2_data,
                   sig_sync => ps2_data_sync );

    -- PS2 Clock Edge Detection
    ps2_clk_negedge <= (not ps2_clk_sync) and edge_ff;
    process(clk, rst)
    begin
        if rst = '1' then
            edge_ff <= '0';
        elsif rising_edge(clk) then
            edge_ff <= ps2_clk_sync;
        end if;
    end process;

    -- Set output data
    rx_data_o <= rxdata(8 downto 1);

    -- Parity bit logic
    parity_bit <= not xor_reduce(rxdata(8 downto 1));

    -- Recieve state machine
    process(clk, rst) 
    begin
        if rst = '1' then
            state <= IDLE;
            rx_done <= '0';
            bit_cnt <= to_unsigned(0, 4);
            rxdata <= (others => '0');
        elsif rising_edge(clk) then
            rx_done <= '0';

            case state is
                when IDLE =>
                    bit_cnt <= to_unsigned(0, 4);
                    if ps2_clk_negedge = '1' and ps2_data_sync = '0' then
                        state <= RECEIVE;
                        rxdata <= ps2_data_sync & rxdata(10 downto 1);
                        bit_cnt <= bit_cnt + 1; -- start bit
                    end if;

                when RECEIVE =>
                    if ps2_clk_negedge = '1' then
                        rxdata <= ps2_data_sync & rxdata(10 downto 1);
                        if bit_cnt = to_unsigned(10, 4) then
                            state <= DONE;
                        else
                            bit_cnt <= bit_cnt + 1; -- data bits + parity + stop
                        end if;
                    end if;

                when DONE =>
                    if parity_bit = rxdata(9) then
                        rx_done <= '1';
                    end if;
                    state <= IDLE;

                when others =>
                    state <= IDLE;

            end case;

        end if;
    end process;

end rtl;
