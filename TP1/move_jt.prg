MODULE move_jt

    CONST num MAX_SAMPLES:=1000;
    VAR num idx_axis;
    VAR jointtarget joints;
    
    CONST string tray_filename:="UBA/pablo/teorica/jt_ref.txt";
    CONST errnum ERR_FILE_BC := 1;
    CONST errnum ERR_LOADING := 2;
    VAR num jt_cant:=0;
    CONST num MAX_JT:=150;
    VAR jointtarget jt{MAX_JT};
    VAR jointtarget jt_cero:=[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]]; 
    VAR num Tmove{MAX_JT};
    VAR robtarget rt_fin;

    PROC MAIN()
    	MoveAbsJ jt_cero,v200,fine,tool0;	
    	TPErase;
        cargarTray;
        recorrerTray;
    ENDPROC
    
    PROC recorrerTray()
        VAR num jt_idx;
        VAR num idx_ciclo;
        VAR bool flag_adquiriendo:=FALSE;
        
        !MoveAbsJ jt_cero,v200,fine,tool0;
        MoveAbsJ jt{1},v200,fine,tool0;
        WaitTime(1); 
        
        FOR jt_idx FROM 1 TO jt_cant STEP 4 DO
            MoveAbsJ jt{jt_idx},v5000\T:=Tmove{jt_idx},z10,tool0;
        ENDFOR
        MoveAbsJ jt{jt_cant},v100,fine,tool0;
        
        rt_fin := CRobT();        
        MoveAbsJ jt{1},v200,fine,tool0;
        
        ConfL \Off;
        ConfJ \Off;
        !SingArea \Wrist;
        MoveL rt_fin,v1000,fine,tool0;
        !SingArea \Off;        
    ENDPROC

        
    PROC cargarTray()
        VAR iodev tray_file;
        VAR string text;
        VAR num n_campo:=0;        
        VAR bool flag_conv;
        VAR num nval;            

        Open "HOME:" \File:= tray_filename, tray_file \Read;
        
        jt_cant:=1;
        TPWrite "Leyendo archivo de trayectoria";
        WHILE text<>EOF AND jt_cant<=MAX_JT DO
            n_campo:=0;
            jt{jt_cant}:=[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
            WHILE text<>EOF AND n_campo<7 DO            
                text:=ReadStr (tray_file \Delim:="," \RemoveCR \DiscardHeaders );
                nval:=-1;
                IF text<>EOF THEN                    
                    flag_conv := StrToVal(text,nval);
                    TEST n_campo
                        CASE 0:
                            Tmove{jt_cant} := nval;
                        CASE 1:
                            jt{jt_cant}.robax.rax_1 := nval;
                        CASE 2:
                            jt{jt_cant}.robax.rax_2 := nval; 
                        CASE 3:
                            jt{jt_cant}.robax.rax_3 := nval;
                        CASE 4:
                            jt{jt_cant}.robax.rax_4 := nval;
                        CASE 5:
                            jt{jt_cant}.robax.rax_5 := nval;
                        CASE 6:
                            jt{jt_cant}.robax.rax_6 := nval;
                    ENDTEST
                    !TPWrite text \Bool:=flag_conv;
                    IF NOT flag_conv RAISE ERR_FILE_BC;                    
                    Incr n_campo;
                ENDIF
            ENDWHILE
            IF text<>EOF Incr jt_cant;
        ENDWHILE              
        jt_cant:=jt_cant-1;
        TPWrite "Joint Targets leidos: " \Num:=jt_cant;
    ERROR
        IF ERRNO=ERR_FILEOPEN THEN
            TPWrite "Cargando Joint Targets de trayectoria";
            TPWrite "Archivo de targets "; 
            TPWrite tray_filename ;
            TPWrite "No encontrado";
        ELSEIF ERRNO=ERR_FILE_BC THEN
            TPwrite "Cargando Joint Targets de trayectoria";
            TPwrite "Archivo de targets "; 
            TPwrite tray_filename;
            TPwrite "Corrupto";
        ENDIF
        !RAISE ERR_LOADING;
        RAISE;
        RETURN;
    ENDPROC    
    

ENDMODULE
