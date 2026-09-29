#!/usr/bin/env python3
"""Genera figuras y tablas científicas sin modificar el runtime ni los rosbags.

Fuentes: CSV controlados en ``src/results`` y resultados/rosbags conservados en
``optimization_results``. Los archivos derivados se escriben exclusivamente en
``hotel_path_planner/docs/analysis_assets``.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


SCRIPT = Path(__file__).resolve()
SRC_ROOT = SCRIPT.parents[3]
WORKSPACE = SRC_ROOT.parent
ASSETS = SCRIPT.parents[1] / "analysis_assets"
RESULTS = SRC_ROOT / "results"
OPTIMIZATION = WORKSPACE / "optimization_results"
RUN1 = OPTIMIZATION / "run_001_original_speed_baseline"
RUN2 = OPTIMIZATION / "run_002_adaptive_double_speed"

COLORS = {"Dijkstra": "#277DA1", "Hybrid A*": "#F8961E",
          "Pure Pursuit": "#43AA8B", "LQR": "#F94144",
          "Run 001": "#577590", "Run 002": "#F3722C"}


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        raise ValueError(f"No hay filas para {path}")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def f(row: dict[str, str], field: str) -> float:
    return float(row[field])


def mean(rows: list[dict[str, str]], field: str) -> float:
    return float(np.mean([f(row, field) for row in rows]))


def style(axis: plt.Axes) -> None:
    axis.grid(True, alpha=0.25, linewidth=0.7)
    axis.spines[["top", "right"]].set_visible(False)


def save(figure: plt.Figure, name: str) -> None:
    figure.tight_layout()
    figure.savefig(ASSETS / name, dpi=240, bbox_inches="tight")
    plt.close(figure)


def controlled_planner_rows() -> tuple[list[dict[str, str]], list[dict[str, str]]]:
    offline = [row for row in read_csv(RESULTS / "planner_results.csv")
               if row["execution_mode"] == "offline_grid_benchmark"]
    gazebo = [row for row in read_csv(RESULTS / "gazebo_planner_results.csv")
              if row["scenario"] == "gazebo_map2_straight_repeated"]
    return offline, gazebo


def plot_planners(offline: list[dict[str, str]]) -> None:
    scenarios = ["open", "orientation_change", "narrow", "obstacle_manoeuvre"]
    labels = ["Abierto", "Cambio de\norientación", "Estrecho", "Obstáculo"]
    methods = [("dijkstra_pp_2602_hotel", "Dijkstra"),
               ("hybrid_astar_pp_2602_hotel", "Hybrid A*")]
    x = np.arange(len(scenarios)); width = 0.36
    figure, axes = plt.subplots(1, 2, figsize=(12.4, 4.7))
    for offset, (method, label) in zip((-width / 2, width / 2), methods):
        grouped = [[row for row in offline if row["scenario"] == scenario and row["method"] == method]
                   for scenario in scenarios]
        axes[0].bar(x + offset, [mean(rows, "planning_time_ms") for rows in grouped], width,
                    label=label, color=COLORS[label])
        axes[1].bar(x + offset, [mean(rows, "path_length_m") for rows in grouped], width,
                    label=label, color=COLORS[label])
    axes[0].set(title="Tiempo de planificación (benchmark offline, n=3)", ylabel="Tiempo [ms]")
    axes[1].set(title="Longitud del path (benchmark offline, n=3)", ylabel="Longitud [m]")
    for axis in axes:
        axis.set_xticks(x, labels); axis.legend(); style(axis)
    save(figure, "01_planner_comparison.png")


def plot_trackers(gazebo: list[dict[str, str]]) -> None:
    by_name = {"Pure Pursuit": next(row for row in gazebo if row["method"].startswith("pure")),
               "LQR": next(row for row in gazebo if row["method"].startswith("lqr"))}
    metrics = [("cte_rmse_m", "RMSE lateral", "m"),
               ("completion_time_s", "Tiempo", "s"),
               ("mean_abs_delta_omega_rad_s", "Media |Δω|", "rad/s")]
    figure, axes = plt.subplots(1, 3, figsize=(12.4, 4.4))
    for axis, (field, title, unit) in zip(axes, metrics):
        names = list(by_name); values = [f(by_name[name], field) for name in names]
        bars = axis.bar(names, values, color=[COLORS[name] for name in names])
        for bar, value in zip(bars, values):
            axis.text(bar.get_x() + bar.get_width()/2, value, f"{value:.4f}",
                      ha="center", va="bottom", fontsize=9)
        axis.set(title=f"{title} (Gazebo, n=1)", ylabel=unit); style(axis)
    save(figure, "02_tracker_comparison.png")


def numeric_trace(path: Path) -> dict[str, np.ndarray]:
    rows = read_csv(path)
    output: dict[str, np.ndarray] = {}
    for key in rows[0]:
        try:
            output[key] = np.asarray([float(row[key]) for row in rows], dtype=float)
        except (ValueError, TypeError):
            continue
    return output


def decimate(size: int, maximum: int = 6000) -> np.ndarray:
    return np.arange(size)[::max(1, math.ceil(size / maximum))]


def plot_races(run1: dict[str, np.ndarray], run2: dict[str, np.ndarray],
               profile1: dict[str, np.ndarray], profile2: dict[str, np.ndarray],
               result1: dict[str, Any], result2: dict[str, Any]) -> None:
    for name, field, ylabel, title in [
        ("03_run001_run002_speed.png", "actual_v_m_s", "Velocidad real [m/s]", "Velocidad real durante la misión"),
        ("04_run001_run002_lateral_error.png", "lateral_error_m", "Error lateral [m]", "Error lateral firmado durante la misión"),
    ]:
        figure, axis = plt.subplots(figsize=(11.2, 4.6))
        for trace, label in ((run1, "Run 001"), (run2, "Run 002")):
            index = decimate(len(trace["time_s"]))
            time = trace["time_s"] - trace["time_s"][0]
            values = np.abs(trace[field]) if field == "actual_v_m_s" else trace[field]
            axis.plot(time[index], values[index], label=label, color=COLORS[label], linewidth=1.0)
        axis.set(title=title, xlabel="Tiempo desde inicio [s]", ylabel=ylabel)
        axis.legend(); style(axis); save(figure, name)

    figure, axes = plt.subplots(1, 2, figsize=(12.4, 5.5))
    for axis, trace, profile, label in ((axes[0], run1, profile1, "Run 001"),
                                        (axes[1], run2, profile2, "Run 002")):
        index = decimate(len(trace["x_m"]), 7000)
        axis.plot(profile["x_m"], profile["y_m"], "--", color="0.25", linewidth=1.0,
                  label="Path planificado")
        axis.plot(trace["x_m"][index], trace["y_m"][index], color=COLORS[label], linewidth=1.0,
                  label="Trayectoria ejecutada")
        axis.set(title=label, xlabel="x [m]", ylabel="y [m]")
        axis.set_aspect("equal", adjustable="box"); axis.legend(fontsize=8); style(axis)
    save(figure, "05_run001_run002_xy.png")

    laps1 = [result1["lap_1"]["time_s"], result1["lap_2"]["time_s"]]
    laps2 = [result2["lap_1"]["time_s"], result2["lap_2"]["time_s"]]
    x = np.arange(2); width = 0.36
    figure, axis = plt.subplots(figsize=(7.4, 4.8))
    axis.bar(x-width/2, laps1, width, label="Run 001", color=COLORS["Run 001"])
    axis.bar(x+width/2, laps2, width, label="Run 002", color=COLORS["Run 002"])
    axis.set(xticks=x, xticklabels=["Vuelta 1", "Vuelta 2"], ylabel="Tiempo [s]",
             title="Tiempo por vuelta"); axis.legend(); style(axis)
    save(figure, "06_lap_times.png")

    rmse = [result1["lateral_rmse_m"], result2["lateral_rmse_m"]]
    maximum = [result1["lateral_max_error_m"], result2["lateral_max_error_m"]]
    figure, axis = plt.subplots(figsize=(7.6, 4.8))
    axis.bar(x-width/2, rmse, width, label="RMSE", color="#4D908E")
    axis.bar(x+width/2, maximum, width, label="Máximo", color="#F9844A")
    axis.set(xticks=x, xticklabels=["Run 001", "Run 002"], ylabel="Error [m]",
             title="Error lateral: RMSE y máximo"); axis.legend(); style(axis)
    save(figure, "07_rmse_comparison.png")


def native_adaptive_limits(bag: Path) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    topics = {
        "curvature": "/path_tracking/speed_limit_curvature",
        "preview": "/path_tracking/speed_limit_preview",
        "omega": "/path_tracking/speed_limit_omega",
        "lateral_error": "/path_tracking/speed_limit_lateral_error",
        "target": "/path_tracking/target_speed",
    }
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = set(topics.values()) - set(types)
    if missing:
        raise RuntimeError(f"Topics adaptativos ausentes: {sorted(missing)}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics.values())))
    values: dict[str, list[tuple[float, float]]] = {name: [] for name in topics}
    reverse = {topic: name for name, topic in topics.items()}
    while reader.has_next():
        topic, serialized, stamp_ns = reader.read_next()
        msg = deserialize_message(serialized, get_message(types[topic]))
        values[reverse[topic]].append((stamp_ns * 1e-9, float(msg.data)))
    return {name: (np.asarray([p[0] for p in series]), np.asarray([p[1] for p in series]))
            for name, series in values.items()}


def plot_and_export_limits(native: dict[str, tuple[np.ndarray, np.ndarray]]) -> None:
    names = ["curvature", "preview", "omega", "lateral_error"]
    labels = {"curvature": "Curvatura actual", "preview": "Curvatura futura",
              "omega": "Capacidad angular", "lateral_error": "Error lateral"}
    colors = {"curvature": "#277DA1", "preview": "#F8961E",
              "omega": "#43AA8B", "lateral_error": "#F94144"}
    base_time = native["curvature"][0]
    origin = base_time[0]
    matrix = np.vstack([native[name][1] for name in names])
    seconds = {name: 0.0 for name in names}
    dominant = np.argmin(matrix, axis=0)
    for i, dt in enumerate(np.diff(base_time)):
        if 0.0 < dt <= 0.2:
            winners = np.flatnonzero(np.abs(matrix[:, i] - matrix[dominant[i], i]) <= 1e-5)
            for winner in winners:
                seconds[names[int(winner)]] += float(dt) / len(winners)
    total = sum(seconds.values())

    figure, axes = plt.subplots(2, 1, figsize=(11.2, 7.0), gridspec_kw={"height_ratios": [3, 1]})
    for name in names:
        times, values = native[name]; index = decimate(len(times), 7000)
        axes[0].plot(times[index]-origin, values[index], label=labels[name], color=colors[name], linewidth=.9)
    times, values = native["target"]; index = decimate(len(times), 7000)
    axes[0].plot(times[index]-origin, values[index], color="black", linewidth=.9, label="Velocidad objetivo")
    axes[0].set(title="Run 002: límites internos de Adaptive Pure Pursuit",
                xlabel="Tiempo de diagnóstico [s]", ylabel="Límite de velocidad [m/s]")
    axes[0].legend(ncol=3, fontsize=8); style(axes[0])
    percentages = [100.0*seconds[name]/total for name in names]
    axes[1].barh([labels[name] for name in names], percentages,
                 color=[colors[name] for name in names])
    axes[1].set(title="Límite mínimo dominante", xlabel="Tiempo como límite dominante [%]")
    style(axes[1]); save(figure, "08_adaptive_speed_limits.png")

    rows = []
    for index in range(len(base_time)):
        row = {"time_s": base_time[index]-origin}
        for name in names:
            row[f"speed_limit_{name}_m_s"] = native[name][1][index]
        row["dominant_limit"] = names[int(dominant[index])]
        rows.append(row)
    write_csv(ASSETS / "adaptive_limits.csv", rows)
    write_csv(ASSETS / "adaptive_limit_dominance.csv", [
        {"limit": name, "seconds": seconds[name], "percent": 100.0*seconds[name]/total,
         "source": "native Float32 diagnostics in run_002 MCAP"} for name in names])


def box(axis: plt.Axes, x: float, y: float, text: str, color: str = "#E9F2F9",
        width: float = 0.20, height: float = 0.10) -> None:
    patch = plt.Rectangle((x-width/2, y-height/2), width, height, facecolor=color,
                          edgecolor="#294C60", linewidth=1.2)
    axis.add_patch(patch); axis.text(x, y, text, ha="center", va="center", fontsize=9)


def arrow(axis: plt.Axes, start: tuple[float, float], end: tuple[float, float], label: str = "") -> None:
    axis.annotate("", xy=end, xytext=start,
                  arrowprops={"arrowstyle": "->", "lw": 1.4, "color": "#294C60"})
    if label:
        axis.text((start[0]+end[0])/2, (start[1]+end[1])/2+.025, label,
                  ha="center", fontsize=8, color="#294C60")


def plot_reactive_state_machine() -> None:
    figure, axis = plt.subplots(figsize=(10.5, 4.2)); axis.set(xlim=(0, 1), ylim=(0, 1)); axis.axis("off")
    points = [(0.12, .45), (.38, .45), (.65, .45), (.90, .45)]
    labels = ["TRACKING\nAdaptive PP", "AVOIDING\nFTG prioritario", "REJOINING\nPP limitado", "TRACKING\nperfil nominal"]
    for point, label in zip(points, labels): box(axis, *point, label, width=.18, height=.20)
    arrow(axis, (.21,.45), (.29,.45)); axis.text(.25,.62,"bloqueo ≥ 0.30 s",ha="center",fontsize=8)
    arrow(axis, (.47,.45), (.56,.45)); axis.text(.515,.62,"libre ≥ 0.60 s\ny evitando ≥ 0.80 s",ha="center",fontsize=8)
    arrow(axis, (.74,.45), (.81,.45)); axis.text(.775,.62,"|eᵧ| < 0.15 m, |eψ| < 0.22 rad\ndurante 0.80 s",ha="center",fontsize=8)
    axis.set_title("Máquina de estados reactiva implementada", fontsize=13)
    save(figure, "09_reactive_state_machine.png")


def plot_architecture() -> None:
    figure, axis = plt.subplots(figsize=(13, 8)); axis.set(xlim=(0, 1), ylim=(0, 1)); axis.axis("off")
    nodes = {
        "Mapa": (.10,.88), "AMCL": (.10,.70), "map→odom": (.10,.52),
        "Odom ruedas\n+ IMU": (.31,.88), "EKF": (.31,.70), "odom→base_link": (.31,.52),
        "Waypoints": (.53,.90), "Dijkstra /\nHybrid A*": (.53,.72), "/planned_path": (.53,.54),
        "PP / LQR /\nAdaptive PP": (.53,.36), "/cmd_vel_nav": (.48,.18),
        "LiDAR + corredor\ndel path": (.78,.90), "Supervisor\nreactivo": (.78,.72), "FTG": (.78,.54),
        "/cmd_vel_gap": (.78,.36), "twist_mux": (.64,.18), "AEB": (.79,.18),
        "DiffDrive\nController": (.93,.18),
    }
    for label, point in nodes.items():
        width = .12 if label in {"/cmd_vel_nav", "twist_mux", "AEB", "DiffDrive\nController"} else .15
        box(axis, *point, label, width=width, height=.09,
            color="#FFF3D6" if label in {"AEB", "Supervisor\nreactivo"} else "#E9F2F9")
    links = [("Mapa","AMCL"),("AMCL","map→odom"),("Odom ruedas\n+ IMU","EKF"),("EKF","odom→base_link"),
             ("Waypoints","Dijkstra /\nHybrid A*"),("Dijkstra /\nHybrid A*","/planned_path"),
             ("/planned_path","PP / LQR /\nAdaptive PP"),("PP / LQR /\nAdaptive PP","/cmd_vel_nav"),
             ("LiDAR + corredor\ndel path","Supervisor\nreactivo"),("Supervisor\nreactivo","FTG"),
             ("FTG","/cmd_vel_gap"),("/cmd_vel_nav","twist_mux"),("/cmd_vel_gap","twist_mux"),
             ("twist_mux","AEB"),("AEB","DiffDrive\nController")]
    for left, right in links:
        a, b = nodes[left], nodes[right]
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx,dy); ux,uy = dx/length,dy/length
        arrow(axis, (a[0]+.055*ux,a[1]+.045*uy), (b[0]-.055*ux,b[1]-.045*uy))
    axis.text(.20,.42,"TF: map → odom → base_link",ha="center",fontsize=10,color="#294C60")
    axis.set_title("Arquitectura final de planificación, seguimiento y seguridad", fontsize=14)
    save(figure, "10_full_architecture.png")


def export_tables(offline: list[dict[str, str]], gazebo_planners: list[dict[str, str]],
                  gazebo_trackers: list[dict[str, str]], result1: dict[str, Any],
                  result2: dict[str, Any]) -> None:
    def group(method: str, scenario: str) -> list[dict[str, str]]:
        return [row for row in offline if row["method"] == method and row["scenario"] == scenario]
    d, h = "dijkstra_pp_2602_hotel", "hybrid_astar_pp_2602_hotel"
    planner = []
    for scenario in ["open", "orientation_change", "narrow", "obstacle_manoeuvre"]:
        dr, hr = group(d, scenario), group(h, scenario)
        planner.extend([
            {"metric": f"planning_time_{scenario}", "unit": "ms", "dijkstra": mean(dr,"planning_time_ms"),
             "hybrid_astar": mean(hr,"planning_time_ms"), "observation": "media offline n=3"},
            {"metric": f"path_length_{scenario}", "unit": "m", "dijkstra": mean(dr,"path_length_m"),
             "hybrid_astar": mean(hr,"path_length_m"), "observation": "mismo escenario y OccupancyGrid"},
            {"metric": f"terminal_yaw_error_{scenario}", "unit": "rad", "dijkstra": mean(dr,"terminal_yaw_error_rad"),
             "hybrid_astar": mean(hr,"terminal_yaw_error_rad"), "observation": "orientación no pertenece al estado Dijkstra"},
        ])
    dg = [r for r in gazebo_planners if r["method"] == d]; hg = [r for r in gazebo_planners if r["method"] == h]
    planner.append({"metric":"gazebo_straight_planning_time", "unit":"ms", "dijkstra":mean(dg,"planning_time_ms"),
                    "hybrid_astar":mean(hg,"planning_time_ms"), "observation":"tramo recto abierto, n=3"})
    write_csv(ASSETS / "planner_comparison.csv", planner)

    pp = next(row for row in gazebo_trackers if row["method"].startswith("pure"))
    lqr = next(row for row in gazebo_trackers if row["method"].startswith("lqr"))
    tracker = []
    for field, unit, note in [
        ("cte_rmse_m","m","Gazebo recto controlado, n=1"), ("cte_max_m","m","Gazebo recto controlado, n=1"),
        ("heading_rmse_rad","rad","Gazebo recto controlado, n=1"), ("completion_time_s","s","Gazebo recto controlado, n=1"),
        ("final_goal_error_m","m","Gazebo recto controlado, n=1"),
        ("mean_abs_delta_omega_rad_s","rad/s","suavidad discreta del comando, n=1"),
        ("progress_percent","%","ambos llegaron al goal"), ("omega_saturations","count","sin saturación")]:
        tracker.append({"metric":field,"unit":unit,"pure_pursuit":pp[field],"lqr":lqr[field],"observation":note})
    write_csv(ASSETS / "tracker_comparison.csv", tracker)

    race_fields = [
        ("total_time_s","s"),("lap_1_time_s","s"),("lap_2_time_s","s"),
        ("mean_sampled_velocity_m_s","m/s"),("maximum_velocity_m_s","m/s"),
        ("lateral_rmse_m","m"),("lateral_max_error_m","m"),("distance_travelled_m","m"),
        ("aeb_active_time_s","s"),("aeb_events","count"),("minimum_lidar_distance_m","m"),
        ("path_progress_percent","%")]
    race = []
    for field, unit in race_fields:
        if field == "lap_1_time_s": a,b=result1["lap_1"]["time_s"],result2["lap_1"]["time_s"]
        elif field == "lap_2_time_s": a,b=result1["lap_2"]["time_s"],result2["lap_2"]["time_s"]
        else: a,b=result1[field],result2[field]
        race.append({"metric":field,"unit":unit,"run_001":a,"run_002":b,"run_002_minus_run_001":b-a,
                     "source":"results.yaml + monitor_trace.csv"})
    race.append({"metric":"time_reduction","unit":"%","run_001":0.0,
                 "run_002":100*(result1["total_time_s"]-result2["total_time_s"])/result1["total_time_s"],
                 "run_002_minus_run_001":100*(result1["total_time_s"]-result2["total_time_s"])/result1["total_time_s"],
                 "source":"cálculo end-to-end; paths no idénticos"})
    write_csv(ASSETS / "race_comparison.csv", race)

    params = [
        ("controller","pure_pursuit","adaptive_pure_pursuit","selección del nodo"),
        ("control_rate","25 Hz","25 Hz","loop de control"),
        ("nominal_linear_speed","0.45 m/s","0.90 m/s","velocidad nominal"),
        ("max_linear_speed","0.50 m/s","1.00 m/s","límite del controlador"),
        ("max_angular_speed","2.00 rad/s","4.00 rad/s","límite angular"),
        ("lookahead","clamp(0.60+1.30|v|,0.30,1.20) m","clamp((0.60+0.60|v_prev|)/(1+0.60|κ_preview|),0.30,1.20) m","ley activa"),
        ("curvature_preview_distance","no aplica","1.20 m","máximo |κ| futuro"),
        ("linear_acceleration_limit","no explícito","0.70 m/s²","rate limiter"),
        ("linear_deceleration_limit","no explícito","1.40 m/s²","rate limiter"),
        ("wheel_angular_limit","10 rad/s","20 rad/s","metadatos de run")]
    write_csv(ASSETS / "main_parameters.csv", [
        {"parameter":p,"standard_pure_pursuit":a,"adaptive_pure_pursuit":b,"function":note}
        for p,a,b,note in params])

    reactive = [
        ("corridor_half_width","0.35 m","0.20 m robot half-width + 0.15 m margin"),
        ("obstacle_activation_distance","2.20 m","alcance máximo de activación"),
        ("minimum_returns","3","persistencia espacial"),("blocking_persistence","0.30 s","TRACKING→AVOIDING"),
        ("clear_persistence","0.60 s","AVOIDING→REJOINING"),("minimum_avoiding_time","0.80 s","evita conmutación inmediata"),
        ("preview_base","1.10 m","preview dinámico"),("preview_speed_gain","0.55 s","término lineal con velocidad"),
        ("preview_range","1.00–2.50 m","clamp"),("braking_deceleration","1.40 m/s²","distancia de frenado"),
        ("ftg_fov","120 deg","sector frontal"),("ftg_bubble_radius","0.35 m","bubble base"),
        ("ftg_min_gap_width","0.55 m","ancho físico mínimo"),("ftg_max_linear_speed","0.68 m/s","mando reactivo"),
        ("ftg_max_angular_speed","2.00 rad/s","mando reactivo"),("rejoin_max_linear_speed","0.68 m/s","límite temporal"),
        ("rejoin_lateral_error","0.15 m","condición estable"),("rejoin_heading_error","0.22 rad","condición estable")]
    write_csv(ASSETS / "reactive_parameters.csv", [
        {"parameter":p,"value":v,"function":purpose,
         "source":"hotel_ttc_follow_the_gap/config/reactive_avoidance.yaml"} for p,v,purpose in reactive])


def main() -> None:
    ASSETS.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": 9.5, "axes.titlesize": 11.5, "axes.labelsize": 10,
                         "figure.facecolor": "white", "axes.facecolor": "white"})
    offline, gazebo_planners = controlled_planner_rows()
    gazebo_trackers = read_csv(RESULTS / "gazebo_tracker_results.csv")
    data1 = load_yaml(RUN1 / "results.yaml")["result"]
    data2 = load_yaml(RUN2 / "results.yaml")["result"]
    trace1, trace2 = numeric_trace(RUN1 / "monitor_trace.csv"), numeric_trace(RUN2 / "monitor_trace.csv")
    profile1, profile2 = numeric_trace(RUN1 / "path_profile.csv"), numeric_trace(RUN2 / "path_profile.csv")
    plot_planners(offline); plot_trackers(gazebo_trackers)
    plot_races(trace1, trace2, profile1, profile2, data1, data2)
    bag = next((RUN2 / "rosbag").glob("*"))
    plot_and_export_limits(native_adaptive_limits(bag))
    plot_reactive_state_machine(); plot_architecture()
    export_tables(offline, gazebo_planners, gazebo_trackers, data1, data2)
    print(f"Artefactos generados en {ASSETS}")


if __name__ == "__main__":
    main()
