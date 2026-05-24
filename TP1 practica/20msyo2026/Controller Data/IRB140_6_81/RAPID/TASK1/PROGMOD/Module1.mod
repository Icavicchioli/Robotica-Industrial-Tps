MODULE Module1
    CONST robtarget t_perno:=[[0,0,0],[0,0,1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_10:=[[65,-35,150],[0,0,1,0],[-1,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget t_agujero:=[[65,65,0],[0,0,1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_10_2:=[[65,65,150],[0,0,1,0],[-1,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    !***********************************************************
    !
    ! Module:  Module1
    !
    ! Description:
    !   <Insert description here>
    !
    ! Author: Ignacio Cavicchioli
    !
    ! Version: 1.0
    !
    !***********************************************************
    
    
    !***********************************************************
    !
    ! Procedure main
    !
    !   This is the entry point of your program
    !
    !***********************************************************
    PROC main()
        !Add your code here
    ENDPROC
    PROC Path_10()
        MoveL t_perno,v100,fine,Pinza\WObj:=perno;
        WaitTime\InPos,1;
        SetDO DO_Pinza,1;
        MoveJ Target_10,v100,z10,Pinza\WObj:=mesa;
        MoveJ Target_10_2,v100,z10,Pinza\WObj:=mesa;
        MoveL t_agujero,v30,fine,Pinza\WObj:=mesa;
        SetDO DO_Pinza,0;
    ENDPROC
ENDMODULE