//GLOBAL SLOW CLOCK IS 1HZ, EVERYTHING HAPPENS AT THIS RATE
//00 = RED
//01 = YELLOW
//10 = GREEN
//11 = ALL ON; ERROR OCCURED
//Yellow Time = 3 seconds
//Min Green for HWY, SideRoad and Pedestrian = 6 seconds


//MAIN MODULE; INTERFACES WITH ALL 3 TIMERS

//If both ped and side are waiting, a past record is used to determine which to use first
//(Cannont be both green)
module lab9v2(LEDR, LEDG, SW, KEY, CLOCK_50);
    input SW;       //Car Detect
    input [3:0] KEY;      //Cross Button
    input CLOCK_50; //50Mz, must be divided by 50 million before being used
    output [17:0] LEDR;
    output [8:0] LEDG;

    wire [1:0] hwyState, sideState, pedState;
    wire crossBut, crossButLat, carDetect;

    //TEMP
    wire clk;
    clockDivider myCD(clk, CLOCK_50);

    assign carDetect = SW;      //Car Sensor
    assign crossBut = KEY[0];   //Pedestrian Cross Button (Latching)

    //next to be serviced between highway green cycles
    //0: side road
    //1: pedestrian crossing
    reg nextServiced = 0;
    reg lastServiced = 0;

    //wire [17:0] LEDR = 18'b00000000000000000;    //Default state of red LEDS
    //wire [8:0] LEDG = 9'b000000000;              //Default state of green LEDS

    reg startHwy = 1, triggerSide = 0, triggerPed = 0, crossReset = 0, cycleStarted = 0;

    //Creates timer objects
    hwyTimer myHwy(canChange, hwyState, startHwy, clk);
    sideRdTimer mySide(sideState, clk, triggerSide, carDetect);
    pedTimer myPed(pedState, clk, triggerPed);

    //Creates state to LED converter objects
    two2three hwyConv(LEDR[17], LEDR[16], LEDG[8], hwyState);
    two2three sideConv(LEDR[13], LEDR[12], LEDG[5], sideState);
    two2three pedConv(LEDR[9], LEDR[8], LEDG[2], pedState);

    SRlatch mySR(crossBut, crossReset, crossButLat, nq);

    always @(posedge clk)
    begin
        crossReset = 0;
        if(!canChange && sideState == 'b00 && pedState == 'b00 && !cycleStarted)
        begin
            triggerSide = 0;
            triggerPed = 0;
            startHwy = 1;
        end

        if(hwyState == 'b01 || hwyState == 'b10)
        begin
            if(crossButLat && !carDetect)
                nextServiced = 1;
            else if(!crossButLat && carDetect)
                nextServiced = 0;
            else if(crossButLat && carDetect)
                nextServiced = !lastServiced;
        end

        if((hwyState == 'b01 || hwyState == 'b10 && canChange && (crossButLat || carDetect)) || cycleStarted)
        begin
            cycleStarted = 1;
            startHwy = 0;
            if(hwyState == 'b00)
            begin
                if(nextServiced == 0)   //Side Road
                begin
                    triggerSide = 1;
                end
                if(nextServiced == 1)   //Ped Crossing
                begin
                    triggerPed = 1;
                    crossReset = 1;
                end
                if(sideState != 'b00 || pedState != 'b00)
                begin
                    cycleStarted = 0;
                    triggerPed = 0;
                    triggerSide = 0;
                end
            end
        end
    end
endmodule

module clockDivider(clk, CLOCK_50);
    input CLOCK_50;
    output clk;

    reg clk;

    wire [25:0] q;

    lpm_counter u1(.clock(clk), .q(Q));
    defparam u1.lpm_width = 25;
    defparam u1.lpm.direction = "UP";

    always @(q)
    begin
        if (q == 'd25000000)
            clk = ~clk;
    end
endmodule


//converts 2-bit state data to 3 led outputs
//00 = RED
//01 = YELLOW
//10 = GREEN
//11 = ALL ON; ERROR OCCURED
module two2three(red, yellow, green, state);
    input [1:0] state;
    output red, yellow, green;
    
    reg [2:0] out;
    
    assign red = out[0];
    assign yellow = out[1];
    assign green = out[2];
    
    always @(state)
    begin
        case (state)
            0 : out = 'b001;    //RED
            1 : out = 'b010;    //YELLOW
            2 : out = 'b100;    //GREEN
            3 : out = 'b111;    //ERROR
        endcase
    end
endmodule

module SRlatch(s, r, q, nq);
  input s, r;
  output q, nq;

  nor g1(q, r, nq);
  nor g2(nq, s, q);
endmodule


//Minimum Green time, no maximum time
//start=true: keep as green, start=false: change to yellow then red.
//WILL NOT WAIT for canChange to be high (1) before changing to yellow (then red), external logic required

//hwyTimer TESTED AND WORKING, boots at red if start is low (0)
module hwyTimer(canChange, hwyState, start, clk);
    input start, clk;
    output canChange;
    output [1:0] hwyState;
    
    integer totalGreenTime = 0; //Counter
    integer totalYellowTime = 4; //Counter: Set to 4 so sysetem state starts on red and waits
    integer minGreenTime = 6; //CONSTANT
    integer allowedYellowTime = 3; //CONSTANT
    reg [1:0] hwyState = 'b00;
    reg canChange;

    always @(posedge clk)
    begin
        if(totalGreenTime >= minGreenTime)
            canChange = 'b1;
      	else
          	canChange = 'b0;
        
        if(start)
        begin
            totalYellowTime = 0;
            totalGreenTime = totalGreenTime + 1;
            hwyState = 'b10; //GREEN
        end

        else if (!start)
        begin
            totalGreenTime = 0;
            totalYellowTime = totalYellowTime + 1;
            if (totalYellowTime <= allowedYellowTime)
                hwyState = 'b01; //YELLOW 
            else
                hwyState = 'b00; //RED
        end
    end
endmodule

//TIMER USED FOR SIDE ROAD
//trigger is latching
//carDetect is not latching
//Maximum time for green allowed; 15 seconds is set
//Reset is triggered internally
//sideRdTimer TESTED AND WORKING, boots at red if start is low (0)
module sideRdTimer(sideState, clk, trigger, carDetect);
    input clk, trigger, carDetect;
    output [1:0] sideState;

    integer totalGreenTime = 0; //Counter
    integer totalYellowTime = 4; //Counter: Set to 4 so sysetem state starts on red and waits
    integer minGreenTime = 6; //CONSTANT
    integer allowedYellowTime = 3; //CONSTANT
    integer maxGreenTime = 15; //CONSTANT
    reg [1:0] sideState = 'b00;
    reg latch = 0;
    
    always @(posedge clk)
    begin
        if(trigger)
            latch = 1;
        
        if(latch)
        begin
            totalYellowTime = 0;
            totalGreenTime = totalGreenTime + 1;
            sideState = 'b10; //GREEN
            if(((totalGreenTime >= minGreenTime) && !carDetect ) || (totalGreenTime >= maxGreenTime))
                latch = 0;
        end

        else if (!latch)
        begin
            totalGreenTime = 0;
            totalYellowTime = totalYellowTime + 1;
            if (totalYellowTime <= allowedYellowTime)
                sideState = 'b01; //YELLOW 
            else
                sideState = 'b00; //RED
        end
    end
endmodule

//TIMER USED FOR PEDESTRIAN CROSS
//trigger is latching (syncronous)
//Reset is triggered internally
//pedTimer TESTED AND WORKING, boots at red if start is low (0)
module pedTimer(pedState, clk, trigger);
    input clk, trigger;
    output [1:0] pedState;

    integer totalGreenTime = 0; //Counter
    integer totalYellowTime = 4; //Counter: Set to 4 so sysetem state starts on red and waits
    integer minGreenTime = 6; //CONSTANT
    integer allowedYellowTime = 3; //CONSTANT
    reg [1:0] pedState = 'b00;
    reg latch = 0;
    
    always @(posedge clk)
    begin
        if(trigger)
            latch = 1;
        
        if(latch)
        begin
            totalYellowTime = 0;
            totalGreenTime = totalGreenTime + 1;
            pedState = 'b10; //GREEN
            if(totalGreenTime >= minGreenTime)
                latch = 0;
        end

        else if (!latch)
        begin
            totalGreenTime = 0;
            totalYellowTime = totalYellowTime + 1;
            if (totalYellowTime <= allowedYellowTime)
                pedState = 'b01; //YELLOW 
            else
                pedState = 'b00; //RED
        end
    end
endmodule