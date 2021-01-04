target extended-remote localhost:3333
set print asm-demangle on
monitor arm semihosting enable
monitor reset init
monitor reset halt
load

define step_a_ton
    break start_first_task
    continue
    set $ipx=0
    set $end=256
    while ($ipx != 256)
        info registers
        si
        set $ipx=$ipx+1
    end
end

define silol
    info registers
    si
end

define stackmem
    x/-100x $sp
end
