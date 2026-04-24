MODULE Module1
    CONST robtarget Target_50:=[[506.291651246,0,679.5],[0.5,0,0.866025404,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_30:=[[506.291651246,0,679.5],[0.5,0,0.866025404,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    !***********************************************************
    !
    ! Module:  Module1
    !
    ! Description:
    !   <Insert description here>
    !
    ! Author: q
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
    PROC Path_10()
        MoveJ [[775,100,131],[0,0,1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v500,fine,tool0\WObj:=wobj0;
        MoveL [[775,-100,131],[0,0,1,0],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],vmax,fine,tool0\WObj:=wobj0;
        MoveJ Target_30,v500,z50,tool0\WObj:=wobj0;
    ENDPROC
ENDMODULE