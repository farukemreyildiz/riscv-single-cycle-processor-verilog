`include "CPU.v"
module CPU_tb;

    reg CLK;
    
    SingleCycleCPU SCC (
        .clk(CLK)
    );
    
    initial begin
        CLK = 1'b0;
        forever #5 CLK = ~CLK;
    end

    initial begin
        #200;
        $finish;
    end

    initial begin
        $dumpfile("CPU.vcd");
        $dumpvars(0, CPU_tb);
    end

endmodule
