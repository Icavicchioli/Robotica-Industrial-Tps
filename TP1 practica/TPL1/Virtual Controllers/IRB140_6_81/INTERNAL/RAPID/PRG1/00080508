MODULE Module1
    ! trans rot confdata extex respecto a
    CONST robtarget perno_t_horizontal:=[[0,0,0],[0.707106781,0.707106781,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget agujero_t_horizontal:=[[0,0,0],[0,0,1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget perno_t_vertical:=[[0,0,0],[0,0,1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    ! la rotación de 180° en Y alrededor del wobj del perno

    CONST robtarget agujero_t_vertical:=[[0,0,0],[0.5,0.5,0.5,0.5],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    ! la rotación tomada copiandola de un target rotado a mano en el entorno

    CONST robtarget home_t:=[[624,0,611],[0.49039264,-0.168953175,0.849384968,-0.097545161],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    CONST robtarget perno_alejado2:=[[100,-50,200],[0,1,0,0],[0,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];


    !*********************************************************************
    ! DESCRIPCIÓN GENERAL
    !
    ! Este programa implementa la toma e inserción de un perno en dos
    ! configuraciones posibles:
    !
    !   - Perno horizontal
    !   - Perno vertical
    !
    ! Para cada configuración se define un robtarget de agarre y un
    ! robtarget de inserción. Estos targets no representan únicamente una
    ! posición, sino la pose completa (posición + orientación) que debe
    ! adoptar el conjunto herramienta-pieza durante la operación.
    !
    ! La estación fue construida definiendo World Objects para el perno y
    ! para el agujero. Todos los World Objects fueron alineados con el
    ! sistema de coordenadas global, por lo que sus ejes locales coinciden
    ! con los ejes del mundo.
    !
    ! Los robtargets fueron definidos en el origen de cada World Object,
    ! ajustando únicamente la orientación necesaria para cada operación.
    ! De esta manera, la pose requerida para agarrar o insertar el perno
    ! queda codificada directamente en el target correspondiente.
    !
    ! Las aproximaciones, retiradas y penetraciones se realizan mediante
    ! Offs() aplicados sobre estos targets. Debido a que los World Objects
    ! utilizados tienen la misma orientación que el sistema global, los
    ! desplazamientos se interpretan en las direcciones cartesianas
    ! habituales (X, Y y Z).
    !
    ! Para resolver el caso del perno vertical no se realizó ninguna
    ! redefinición de herramienta (tooldata). La diferencia entre las
    ! configuraciones horizontal y vertical se implementó exclusivamente
    ! mediante la selección de robtargets con la orientación adecuada.
    !
    ! En consecuencia, toda la lógica geométrica del proceso queda
    ! concentrada en la definición de los targets, mientras que la
    ! trayectoria se obtiene aplicando offsets sobre dichas poses de
    ! referencia.
    !
    !*********************************************************************

    CONST num aprox_perno:=100;
    CONST num prof_agarre_perno:=10;
    CONST num aprox_agujero:=50;
    CONST num long_perno:=100;
    CONST num prof_penetracion_agujero:=20;


    PROC main()

        AbrirPinza;

        Home;

        !Trayecto_horizontal;
        Trayecto_vertical;

        Home;




    ENDPROC

    PROC Trayecto_horizontal()
        TomarPerno(perno_t_horizontal);

        ColocarPerno(agujero_t_horizontal);
    ENDPROC

    PROC Trayecto_vertical()
        TomarPerno(perno_t_vertical);

        ColocarPerno(agujero_t_vertical);
    ENDPROC



    PROC TomarPerno(robtarget perno_t)

        MoveJ Offs(perno_t,0,0,aprox_perno),V500,z50,Pinza\WObj:=perno_woj;

        MoveL perno_t,v50,fine,Pinza\WObj:=perno_woj;

        MoveL Offs(perno_t,0,-prof_agarre_perno,0),v50,fine,Pinza\WObj:=perno_woj;

        CerrarPinza;

    ENDPROC


    PROC ColocarPerno(robtarget agujero_t)

        MoveJ perno_alejado2,V500,z50,Pinza\WObj:=perno_woj;

        MoveJ Offs(agujero_t,0,0,long_perno+aprox_agujero),v1000,z100,Pinza\WObj:=Agujero_woj;

        MoveL Offs(agujero_t,0,0,long_perno-prof_penetracion_agujero),v50,fine,Pinza\WObj:=Agujero_woj;

        AbrirPinza;

    ENDPROC



    PROC Home()
        MoveJ home_t,v100,z50,Pinza;
    ENDPROC

    PROC AbrirPinza()
        SetDO DO_Pinza,0;
        WaitTime 0.2;
    ENDPROC

    PROC CerrarPinza()
        SetDO DO_Pinza,1;
        WaitTime 0.2;
    ENDPROC

ENDMODULE
