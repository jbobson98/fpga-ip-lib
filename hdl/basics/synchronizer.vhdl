library ieee;
use ieee.std_logic_1164.all;

entity synchronizer is
    port (
            clk      : in std_logic;
            rst      : in std_logic;
            sig_i    : in std_logic;
            sig_sync : out std_logic
         );
end synchronizer;

architecture rtl of synchronizer is
    signal ff1 : std_logic;
    signal ff2 : std_logic;
begin

    sig_sync <= ff2;

    process(clk, rst) 
    begin
        if rst = '1' then
            ff1 <= '0';
            ff2 <= '0';
        elsif rising_edge(clk) then
            ff1 <= sig_i;
            ff2 <= ff1;
        end if;
    end process;
end rtl;
