MODULE Module1
    PERS tooldata Sopapa:=[TRUE,[[0,-5,103],[1,0,0,0]],[0.5,[0,0,40],[1,0,0,0],0,0,0]];
    CONST robtarget Target_10:=[[595.492267836,-5,628],[0.5,0,0.866025404,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_20:=[[595.492192643,34.78927249,628.000011052],[0.500000018,0.000000003,0.866025394,0.000000005],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget pieces_target:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget pallet_target:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    CONST num X_caja := 130;
    CONST num Y_caja := 65;
    CONST num Z_caja := 15;
    CONST num offs_seguridad := 20;
    
    const num cajas_totales := 9;
    var num CajasEnPalet := 0;    
    
    CONST robtarget pBase:=[[0,0,0],[0,1,0,0],[0,0,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]]; !genérico dado vuelta, Z hacia abajo
    VAR robtarget pDestino;! para poder hacerle offset respecto del resto
    VAR num cajasEnPila;
    
PROC main()

    Path_10;

    FOR CajasEnPalet FROM CajasEnPalet TO cajas_totales DO
        
        cajasEnPila := cajas_totales - CajasEnPalet;

        Pick(cajasEnPila);

        !Place;

    ENDFOR

ENDPROC


    PROC Path_10()
        MoveL Target_10,v1000,z100,Sopapa\WObj:=wobj0;
        MoveL Target_20,v1000,z100,Sopapa\WObj:=wobj0;
        WaitTime(1);
    ENDPROC
    
    
    
PROC Pick(num cantCajas)

    VAR robtarget pPick;

    pPick := Offs(
        pBase,
        X_caja/2,
        Y_caja/2,
        cantCajas * Z_caja
    );

    ! Aproximación con distancia de seguridad
    MoveJ Offs(pPick,0,0,offs_seguridad),v100,z100,Sopapa\WObj:=Piceces_wo;

    ! Descenso hasta la caja
    MoveL pPick,v50,fine,Sopapa\WObj:=Piceces_wo;
          
    waitTime(0.5);
    
    ! La herramienta agarra la caja
    
    waitTime(0.5);

ENDPROC
    
    PROC place()
        MoveL Target_30,v1000,z100,Sopapa\WObj:=Pallet_wo;
    
    ENDPROC
    
    
    

    
    
ENDMODULE