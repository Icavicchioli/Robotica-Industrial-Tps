MODULE Module1
    ! ==========================================
    ! DECLARACIÓN DE HERRAMIENTAS Y PUNTOS FIJOS
    ! ==========================================
    PERS tooldata Sopapa:=[TRUE,[[0,-5,103],[1,0,0,0]],[0.5,[0,0,40],[1,0,0,0],0,0,0]];

    CONST robtarget Target_10:=[[595.492267836,-5,628],[0.5,0,0.866025404,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_20:=[[595.492192643,34.78927249,628.000011052],[0.500000018,0.000000003,0.866025394,0.000000005],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    ! ==========================================
    ! CONSTANTES DE DIMENSIÓN Y CONFIGURACIÓN
    ! ==========================================
    CONST num X_caja:=130;
    CONST num Y_caja:=65;
    CONST num Z_caja:=15;
    CONST num offs_seguridad:=20;
    CONST num cajas_totales:=9;

    ! ==========================================
    ! VARIABLES GLOBALES DE MÓDULO (SIN PARAMETROS)
    ! ==========================================
    VAR num CajasEnPalet:=0;
    VAR num cajasEnPila;
    VAR num nivel_pallet:=0;
    VAR num posicion:=0; ! Ahora es global

    CONST robtarget pBase:=[[0,0,0],[0,1,0,0],[0,0,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    VAR robtarget pPalletBase:=[[195,65,0],[0,1,0,0],[0,0,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    ! ==========================================
    ! RUTINA PRINCIPAL
    ! ==========================================
    PROC main()
        Path_10;

        FOR i FROM 0 TO cajas_totales-1 DO
            CajasEnPalet:=i;
            nivel_pallet:=CajasEnPalet DIV 3;
            cajasEnPila:=cajas_totales-CajasEnPalet;
            
            ! Calculamos la posición en la fila aquí, de forma global
            posicion:=CajasEnPalet MOD 3;

            ! Llamadas limpias, sin pasar ningún argumento entre paréntesis
            Pick;
            Place;
        ENDFOR

        Path_10;
    ENDPROC

    ! ==========================================
    ! PROCEDIMIENTOS AUXILIARES (AHORA LLEER GLOBALES)
    ! ==========================================
    PROC Path_10()
        MoveL Target_10,v1000,z100,Sopapa\WObj:=wobj0;
        MoveL Target_20,v1000,z100,Sopapa\WObj:=wobj0;
        WaitTime(1);
    ENDPROC

    PROC Pick()
        VAR robtarget pPick;

        ! Usa la variable global 'cajasEnPila' directamente
        pPick:=Offs(pBase,X_caja/2,Y_caja/2,cajasEnPila*Z_caja);

        MoveJ Offs(pPick,0,0,offs_seguridad),v100,z100,Sopapa\WObj:=Piceces_wo;
        MoveL pPick,v50,fine,Sopapa\WObj:=Piceces_wo;

        WaitTime(0.5);
        ! [Aquí activas tu ventosa]
        WaitTime(0.5);

        MoveL Offs(pPick,0,0,offs_seguridad),v100,z100,Sopapa\WObj:=Piceces_wo;
    ENDPROC

    PROC Place()
        VAR robtarget pPlace;

        pPlace:=pPalletBase;

        ! Usa la variable global 'nivel_pallet' directamente
        pPlace.trans.z:= (1+nivel_pallet)*Z_caja;

        ! Usa la variable global 'posicion' directamente
        IF posicion=0 THEN
            pPlace:=Offs(pPlace,-X_caja,0,0);
        ELSEIF posicion=1 THEN
            pPlace:=Offs(pPlace,0,0,0);
        ELSEIF posicion=2 THEN
            pPlace:=Offs(pPlace,X_caja,0,0);
        ENDIF

        MoveJ Offs(pPlace,0,0,offs_seguridad),v100,z100,Sopapa\WObj:=Pallet_wo;
        MoveL pPlace,v50,fine,Sopapa\WObj:=Pallet_wo;

        WaitTime(0.5);
        ! [Aquí desactivas tu ventosa]
        WaitTime(0.5);

        MoveL Offs(pPlace,0,0,offs_seguridad),v100,z100,Sopapa\WObj:=Pallet_wo;
    ENDPROC

ENDMODULE