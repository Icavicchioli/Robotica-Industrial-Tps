from __future__ import annotations

"""
Barrido frontal 2D de trayectorias rectas de longitud fija.

Hipotesis por defecto de esta primera version:
- el avance se toma sobre x (`eje_avance = 0`);
- la recta une dos planos paralelos separados 20 cm;
- cada celda de la grilla representa una recta ortogonal a esos planos;
- la metrica principal es la velocidad cartesiana constante maxima compatible
  con los limites articulares del modelo 3R.
"""

import math
from dataclasses import dataclass, field

import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm


# ABB IRB140 normal, no variante T.
# 3-phase: ejes 1-3 = [200, 200, 260] deg/s
QD_MAX_3_TRIFASICA = np.deg2rad(np.array([200.0, 200.0, 260.0]))
QD_MAX_3 = QD_MAX_3_TRIFASICA.copy()

# Se conserva un cubo amplio para evitar que el solver pruebe puntos absurdos.
CUBO_DEFAULT = {
    "x": (-0.75, 0.75),
    "y": (-0.75, 0.75),
    "z": (0.00, 1.30),
}


@dataclass
class ResultadoTrayectoriaRecta:
    success: bool
    razon: str
    q_path: np.ndarray
    p_path: np.ndarray
    dt_segmentos: np.ndarray
    tiempo_total: float
    velocidad_cartesiana: float
    qd_peak: np.ndarray
    sigma_min: float
    q_inicio: np.ndarray | None = None


@dataclass
class ConfigBarridoFrontal:
    eje_avance: int = 0
    avance_centro: float = 0.45
    separacion_planos: float = 0.20
    rango_lateral: tuple[float, float] = (-0.30, 0.30)
    rango_vertical: tuple[float, float] = (0.10, 1.05)
    n_lateral: int = 33
    n_vertical: int = 37
    n_pasos_trayectoria: int = 35
    qd_max: np.ndarray = field(default_factory=lambda: QD_MAX_3.copy())
    cubo: dict[str, tuple[float, float]] = field(default_factory=lambda: dict(CUBO_DEFAULT))
    radio_exclusion_base: float = 0.30
    n_q_catalogo: tuple[int, int, int] = (17, 15, 17)
    n_semillas_ik: int = 8


def crear_irb140_3ejes() -> rtb.DHRobot:
    link1 = rtb.RevoluteDH(alpha=-np.pi / 2, a=0.07, d=0.352)
    link2 = rtb.RevoluteDH(a=0.36, offset=-np.pi / 2)
    link3 = rtb.RevoluteDH(alpha=np.pi / 2, offset=np.pi)

    link4 = rtb.RevoluteDH(d=0.38, alpha=-np.pi / 2)
    link5 = rtb.RevoluteDH(alpha=np.pi / 2)
    link6 = rtb.RevoluteDH(d=0.065)
    tool = link4.A(0.0) * link5.A(0.0) * link6.A(0.0)

    robot = rtb.DHRobot([link1, link2, link3], tool=tool, name="IRB140_3EJES")
    robot.qlim = np.deg2rad(np.array([[-180.0, -100.0, -220.0], [180.0, 100.0, 60.0]]))
    return robot


def en_cubo(p: np.ndarray, cubo: dict[str, tuple[float, float]]) -> bool:
    return (
        cubo["x"][0] <= p[0] <= cubo["x"][1]
        and cubo["y"][0] <= p[1] <= cubo["y"][1]
        and cubo["z"][0] <= p[2] <= cubo["z"][1]
    )


def en_zona_exclusion_base(p: np.ndarray, radio: float) -> bool:
    return float(np.hypot(p[0], p[1])) < radio


def punto_permitido(
    p: np.ndarray,
    cubo: dict[str, tuple[float, float]],
    radio_exclusion_base: float,
) -> bool:
    return en_cubo(p, cubo) and (not en_zona_exclusion_base(p, radio=radio_exclusion_base))


def metricas_jacobiano(robot: rtb.DHRobot, q: np.ndarray) -> dict[str, float]:
    jv = np.asarray(robot.jacob0(q)[:3, :3], dtype=float)
    singulares = np.linalg.svd(jv, compute_uv=False)
    sigma_max = float(singulares[0])
    sigma_min = float(singulares[-1])
    destreza = sigma_min / sigma_max if sigma_max > 1e-9 else 0.0
    return {"sigma_min": sigma_min, "destreza": destreza}


def analizar_tiempos_trayectoria(q_path: np.ndarray, qd_max: np.ndarray):
    if len(q_path) < 2:
        vacio = np.zeros((0, q_path.shape[1] if q_path.ndim == 2 else 3), dtype=float)
        return np.zeros(0), 0.0, vacio, np.zeros(3)
    dq = np.diff(q_path, axis=0)
    dt_constante = max(float(np.max(np.abs(dq) / qd_max[None, :])), 1e-6)
    dt_segmentos = np.full(len(dq), dt_constante, dtype=float)
    qd_segmentos = dq / dt_constante
    qd_peak = np.max(np.abs(qd_segmentos), axis=0)
    tiempo_total = float(np.sum(dt_segmentos))
    return dt_segmentos, tiempo_total, qd_segmentos, qd_peak


def resolver_ik_posicion(
    robot: rtb.DHRobot,
    p_objetivo: np.ndarray,
    q0: np.ndarray | None = None,
    tol: float = 1e-4,
    max_iter: int = 100,
    damping: float = 1e-3,
):
    q = np.zeros(robot.n) if q0 is None else np.array(q0, dtype=float).copy()
    p_objetivo = np.array(p_objetivo, dtype=float)
    try:
        sol = robot.ikine_LM(
            sm.SE3(*p_objetivo),
            q0=q,
            mask=[1, 1, 1, 0, 0, 0],
            joint_limits=True,
        )
        if getattr(sol, "success", False):
            return True, np.asarray(sol.q, dtype=float), "ikine_LM"
    except Exception:
        pass
    q_inf = np.asarray(robot.qlim[0], dtype=float)
    q_sup = np.asarray(robot.qlim[1], dtype=float)
    for _ in range(max_iter):
        p_actual = np.asarray(robot.fkine(q).t, dtype=float)
        error = p_objetivo - p_actual
        if np.linalg.norm(error) < tol:
            return True, q, "jacobiano"
        jv = np.asarray(robot.jacob0(q)[:3, :3], dtype=float)
        sistema = jv @ jv.T + (damping**2) * np.eye(3)
        dq = jv.T @ np.linalg.solve(sistema, error)
        q = np.clip(q + dq, q_inf, q_sup)
    return False, q, "sin_convergencia"


def muestrear_workspace(
    robot: rtb.DHRobot,
    n_q: tuple[int, int, int],
    cubo: dict[str, tuple[float, float]],
    radio_exclusion_base: float,
):
    q1 = np.linspace(robot.qlim[0, 0], robot.qlim[1, 0], n_q[0])
    q2 = np.linspace(robot.qlim[0, 1], robot.qlim[1, 1], n_q[1])
    q3 = np.linspace(robot.qlim[0, 2], robot.qlim[1, 2], n_q[2])
    qs, pos = [], []
    for a1 in q1:
        for a2 in q2:
            for a3 in q3:
                q = np.array([a1, a2, a3], dtype=float)
                p = np.asarray(robot.fkine(q).t, dtype=float)
                if not punto_permitido(p, cubo, radio_exclusion_base):
                    continue
                qs.append(q)
                pos.append(p)
    return {"q": np.asarray(qs), "pos": np.asarray(pos)}


def construir_semillas_ik(
    robot: rtb.DHRobot,
    catalogo: dict[str, np.ndarray],
    p_inicio: np.ndarray,
    n_semillas: int,
) -> list[np.ndarray]:
    semillas = []
    if len(catalogo["q"]) > 0:
        dist2 = np.sum((catalogo["pos"] - p_inicio[None, :]) ** 2, axis=1)
        n_local = min(n_semillas, len(dist2))
        idx = np.argpartition(dist2, n_local - 1)[:n_local]
        idx = idx[np.argsort(dist2[idx])]
        semillas.extend(catalogo["q"][idx])

    q_media = 0.5 * (robot.qlim[0] + robot.qlim[1])
    semillas.extend(
        [
            q_media,
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, -np.pi / 4, np.pi / 3]),
            np.array([0.0, np.pi / 6, -np.pi / 2]),
        ]
    )

    semillas_unicas: list[np.ndarray] = []
    for q in semillas:
        if not any(np.allclose(q, q_existente, atol=1e-6) for q_existente in semillas_unicas):
            semillas_unicas.append(np.asarray(q, dtype=float))
    return semillas_unicas


def construir_punto_linea(
    eje_avance: int,
    avance: float,
    lateral: float,
    vertical: float,
) -> np.ndarray:
    restantes = [idx for idx in range(3) if idx != eje_avance]
    p = np.zeros(3, dtype=float)
    p[eje_avance] = avance
    p[restantes[0]] = lateral
    p[restantes[1]] = vertical
    return p


def evaluar_trayectoria_recta(
    robot: rtb.DHRobot,
    p_inicio: np.ndarray,
    p_fin: np.ndarray,
    semillas_ik: list[np.ndarray],
    n_pasos_trayectoria: int,
    qd_max: np.ndarray,
    cubo: dict[str, tuple[float, float]],
    radio_exclusion_base: float,
) -> ResultadoTrayectoriaRecta:
    mejor: ResultadoTrayectoriaRecta | None = None
    puntos = np.linspace(p_inicio, p_fin, n_pasos_trayectoria)

    for q_seed in semillas_ik:
        ok_inicio, q_actual, metodo = resolver_ik_posicion(robot, p_inicio, q0=q_seed)
        if not ok_inicio:
            razon = f"ik_inicio_{metodo}"
            resultado = ResultadoTrayectoriaRecta(
                success=False,
                razon=razon,
                q_path=np.zeros((0, robot.n)),
                p_path=np.zeros((0, 3)),
                dt_segmentos=np.zeros(0),
                tiempo_total=np.inf,
                velocidad_cartesiana=0.0,
                qd_peak=np.zeros(robot.n),
                sigma_min=0.0,
                q_inicio=None,
            )
            if mejor is None:
                mejor = resultado
            continue

        q_path = [q_actual.copy()]
        p_path = [p_inicio.copy()]
        sigma_min = math.inf
        razon_fallo = "ok"

        for p_objetivo in puntos[1:]:
            if not punto_permitido(p_objetivo, cubo, radio_exclusion_base):
                razon_fallo = "zona_no_permitida"
                break
            ok, q_nuevo, metodo = resolver_ik_posicion(robot, p_objetivo, q0=q_actual)
            if not ok:
                razon_fallo = f"ik_trayectoria_{metodo}"
                break
            sigma_min = min(sigma_min, metricas_jacobiano(robot, q_nuevo)["sigma_min"])
            q_actual = q_nuevo
            q_path.append(q_actual.copy())
            p_path.append(np.asarray(p_objetivo, dtype=float))

        if razon_fallo != "ok":
            resultado = ResultadoTrayectoriaRecta(
                success=False,
                razon=razon_fallo,
                q_path=np.asarray(q_path),
                p_path=np.asarray(p_path),
                dt_segmentos=np.zeros(0),
                tiempo_total=np.inf,
                velocidad_cartesiana=0.0,
                qd_peak=np.zeros(robot.n),
                sigma_min=0.0,
                q_inicio=np.asarray(q_path[0]) if q_path else None,
            )
            if mejor is None:
                mejor = resultado
            continue

        q_path_np = np.asarray(q_path)
        p_path_np = np.asarray(p_path)
        dt_segmentos, tiempo_total, _qd_segmentos, qd_peak = analizar_tiempos_trayectoria(q_path_np, qd_max)
        longitud = float(np.linalg.norm(p_fin - p_inicio))
        velocidad = 0.0 if tiempo_total <= 1e-9 else longitud / tiempo_total
        resultado = ResultadoTrayectoriaRecta(
            success=True,
            razon="ok",
            q_path=q_path_np,
            p_path=p_path_np,
            dt_segmentos=dt_segmentos,
            tiempo_total=tiempo_total,
            velocidad_cartesiana=velocidad,
            qd_peak=qd_peak,
            sigma_min=float(sigma_min),
            q_inicio=q_path_np[0],
        )

        if mejor is None:
            mejor = resultado
            continue
        if resultado.success and (
            (not mejor.success)
            or resultado.tiempo_total < mejor.tiempo_total
            or (
                math.isclose(resultado.tiempo_total, mejor.tiempo_total, rel_tol=1e-9, abs_tol=1e-9)
                and resultado.sigma_min > mejor.sigma_min
            )
        ):
            mejor = resultado

    if mejor is None:
        return ResultadoTrayectoriaRecta(
            success=False,
            razon="sin_semillas_ik",
            q_path=np.zeros((0, robot.n)),
            p_path=np.zeros((0, 3)),
            dt_segmentos=np.zeros(0),
            tiempo_total=np.inf,
            velocidad_cartesiana=0.0,
            qd_peak=np.zeros(robot.n),
            sigma_min=0.0,
            q_inicio=None,
        )
    return mejor


def barrer_plano_frontal(
    robot: rtb.DHRobot,
    config: ConfigBarridoFrontal,
):
    laterales = np.linspace(config.rango_lateral[0], config.rango_lateral[1], config.n_lateral)
    verticales = np.linspace(config.rango_vertical[0], config.rango_vertical[1], config.n_vertical)
    avance_inicio = config.avance_centro - 0.5 * config.separacion_planos
    avance_fin = config.avance_centro + 0.5 * config.separacion_planos
    catalogo = muestrear_workspace(
        robot,
        n_q=config.n_q_catalogo,
        cubo=config.cubo,
        radio_exclusion_base=config.radio_exclusion_base,
    )

    tiempos = np.full((len(verticales), len(laterales)), np.inf, dtype=float)
    velocidades = np.zeros_like(tiempos)
    sigma_min = np.zeros_like(tiempos)
    factibles = np.zeros_like(tiempos, dtype=bool)
    razones = np.empty_like(tiempos, dtype=object)
    resultados = np.empty_like(tiempos, dtype=object)

    mejor_resultado: ResultadoTrayectoriaRecta | None = None
    mejor_indices: tuple[int, int] | None = None

    for i_z, vertical in enumerate(verticales):
        for i_y, lateral in enumerate(laterales):
            p_inicio = construir_punto_linea(config.eje_avance, avance_inicio, lateral, vertical)
            p_fin = construir_punto_linea(config.eje_avance, avance_fin, lateral, vertical)
            if not punto_permitido(p_inicio, config.cubo, config.radio_exclusion_base):
                razones[i_z, i_y] = "inicio_no_permitido"
                continue
            if not punto_permitido(p_fin, config.cubo, config.radio_exclusion_base):
                razones[i_z, i_y] = "fin_no_permitido"
                continue

            semillas_ik = construir_semillas_ik(robot, catalogo, p_inicio, config.n_semillas_ik)
            resultado = evaluar_trayectoria_recta(
                robot=robot,
                p_inicio=p_inicio,
                p_fin=p_fin,
                semillas_ik=semillas_ik,
                n_pasos_trayectoria=config.n_pasos_trayectoria,
                qd_max=np.asarray(config.qd_max, dtype=float),
                cubo=config.cubo,
                radio_exclusion_base=config.radio_exclusion_base,
            )

            resultados[i_z, i_y] = resultado
            razones[i_z, i_y] = resultado.razon
            if not resultado.success:
                continue

            factibles[i_z, i_y] = True
            tiempos[i_z, i_y] = resultado.tiempo_total
            velocidades[i_z, i_y] = resultado.velocidad_cartesiana
            sigma_min[i_z, i_y] = resultado.sigma_min

            if mejor_resultado is None:
                mejor_resultado = resultado
                mejor_indices = (i_z, i_y)
                continue
            if (
                resultado.tiempo_total < mejor_resultado.tiempo_total
                or (
                    math.isclose(resultado.tiempo_total, mejor_resultado.tiempo_total, rel_tol=1e-9, abs_tol=1e-9)
                    and resultado.sigma_min > mejor_resultado.sigma_min
                )
            ):
                mejor_resultado = resultado
                mejor_indices = (i_z, i_y)

    return {
        "config": config,
        "laterales": laterales,
        "verticales": verticales,
        "tiempos": tiempos,
        "velocidades": velocidades,
        "sigma_min": sigma_min,
        "factibles": factibles,
        "razones": razones,
        "resultados": resultados,
        "mejor_resultado": mejor_resultado,
        "mejor_indices": mejor_indices,
    }


def graficar_heatmap_barrido(
    barrido: dict[str, object],
    metrica: str = "velocidades",
    mostrar_mejor: bool = True,
):
    config = barrido["config"]
    laterales = barrido["laterales"]
    verticales = barrido["verticales"]
    factibles = barrido["factibles"]
    mejor_indices = barrido["mejor_indices"]

    if metrica == "velocidades":
        datos = np.asarray(barrido["velocidades"], dtype=float)
        titulo = "Velocidad cartesiana maxima en trayectorias rectas de 20 cm"
        cmap = "RdYlGn"
        cbar = "v [m/s]"
    elif metrica == "tiempos":
        datos = np.asarray(barrido["tiempos"], dtype=float)
        titulo = "Tiempo minimo de trayectorias rectas de 20 cm"
        cmap = "RdYlGn_r"
        cbar = "t [s]"
    else:
        raise ValueError("La metrica debe ser 'velocidades' o 'tiempos'.")

    datos_plot = np.where(factibles, datos, np.nan)
    fig, ax = plt.subplots(figsize=(10, 7))
    imagen = ax.imshow(
        datos_plot,
        origin="lower",
        aspect="auto",
        extent=[laterales[0], laterales[-1], verticales[0], verticales[-1]],
        cmap=cmap,
    )
    ax.set_facecolor("#d9d9d9")
    fig.colorbar(imagen, ax=ax, label=cbar)

    if mostrar_mejor and mejor_indices is not None:
        i_z, i_y = mejor_indices
        ax.scatter(
            [laterales[i_y]],
            [verticales[i_z]],
            s=90,
            marker="*",
            color="black",
            label="mejor recta",
        )
        ax.legend()

    nombres = ["x", "y", "z"]
    restantes = [idx for idx in range(3) if idx != config.eje_avance]
    ax.set_xlabel(f"{nombres[restantes[0]]} [m]")
    ax.set_ylabel(f"{nombres[restantes[1]]} [m]")
    ax.set_title(titulo)
    ax.grid(False)
    fig.tight_layout()
    plt.show()


def graficar_mejor_trayectoria(barrido: dict[str, object]):
    mejor = barrido["mejor_resultado"]
    if mejor is None or not mejor.success:
        raise ValueError("No hay trayectoria factible para graficar.")

    config = barrido["config"]
    p = mejor.p_path
    nombres = ["x", "y", "z"]
    restantes = [idx for idx in range(3) if idx != config.eje_avance]
    idx_h = config.eje_avance
    idx_v = restantes[1]

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(p[:, idx_h], p[:, idx_v], color="tab:blue", linewidth=2)
    ax.scatter(
        [p[0, idx_h], p[-1, idx_h]],
        [p[0, idx_v], p[-1, idx_v]],
        color=["tab:orange", "tab:green"],
        s=60,
    )
    ax.set_xlabel(f"{nombres[idx_h]} [m]")
    ax.set_ylabel(f"{nombres[idx_v]} [m]")
    ax.set_title("Perfil de la trayectoria recta mas rapida")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    plt.show()


def imprimir_resumen_barrido(barrido: dict[str, object]):
    config = barrido["config"]
    factibles = np.asarray(barrido["factibles"], dtype=bool)
    mejor = barrido["mejor_resultado"]
    total = factibles.size
    factibles_count = int(np.count_nonzero(factibles))

    print("=== BARRIDO FRONTAL EN DOS PLANOS ===")
    print(f"Eje de avance: {['x', 'y', 'z'][config.eje_avance]}")
    print(f"Planos separados: {config.separacion_planos:.3f} m")
    print(f"Centro de avance: {config.avance_centro:.3f} m")
    print(
        f"Grilla evaluada: {config.n_lateral} x {config.n_vertical} = {total} rectas"
    )
    print(f"Rectas factibles: {factibles_count} ({factibles_count / total:.1%})")

    if mejor is None or not mejor.success:
        print("No se encontro ninguna trayectoria recta factible en la banda frontal.")
        return

    laterales = barrido["laterales"]
    verticales = barrido["verticales"]
    i_z, i_y = barrido["mejor_indices"]
    nombres = ["x", "y", "z"]
    restantes = [idx for idx in range(3) if idx != config.eje_avance]
    coord_lateral = nombres[restantes[0]]
    coord_vertical = nombres[restantes[1]]

    print()
    print("=== MEJOR RECTA ===")
    print(f"{coord_lateral} = {laterales[i_y]:.4f} m")
    print(f"{coord_vertical} = {verticales[i_z]:.4f} m")
    print("Punto inicial [m] =", np.round(mejor.p_path[0], 4))
    print("Punto final   [m] =", np.round(mejor.p_path[-1], 4))
    print(f"Tiempo total minimo [s] = {mejor.tiempo_total:.6f}")
    print(f"Velocidad cartesiana [m/s] = {mejor.velocidad_cartesiana:.6f}")
    print("q inicio [deg] =", np.round(np.rad2deg(mejor.q_inicio), 2))
    print("qdot pico [deg/s] =", np.round(np.rad2deg(mejor.qd_peak), 2))
    print(f"sigma_min sobre la recta = {mejor.sigma_min:.6f}")


def main():
    robot = crear_irb140_3ejes()
    config = ConfigBarridoFrontal()
    barrido = barrer_plano_frontal(robot, config)
    imprimir_resumen_barrido(barrido)
    graficar_heatmap_barrido(barrido, metrica="velocidades")
    graficar_heatmap_barrido(barrido, metrica="tiempos")
    graficar_mejor_trayectoria(barrido)


if __name__ == "__main__":
    main()
